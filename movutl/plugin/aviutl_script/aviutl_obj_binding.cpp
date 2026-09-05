#include <cstring>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/aviutl_script/aviutl_obj_binding.hpp>
#include <string>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

namespace mu::detail {

namespace {

AviUtlObjContext* get_ctx(lua_State* L) { return static_cast<AviUtlObjContext*>(lua_touserdata(L, lua_upvalueindex(1))); }

// AviUtl仕様: (データ, 幅, 高さ)の3値を返す(第1引数"alloc"等は無視、常に現在のimgサイズを返す)
int l_obj_getpixeldata(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img) return 0;
  lua_pushlstring(L, reinterpret_cast<const char*>(img->data()), img->size_in_bytes());
  lua_pushinteger(L, img->width);
  lua_pushinteger(L, img->height);
  return 3;
}

int l_obj_putpixeldata(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img) return 0;
  size_t len       = 0;
  const char* data = luaL_checklstring(L, 1, &len);
  std::memcpy(img->data(), data, std::min(len, img->size_in_bytes()));
  ctx->drawn = true;
  return 0;
}

int l_obj_getpixel(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  lua_pushinteger(L, img ? img->width : 0);
  lua_pushinteger(L, img ? img->height : 0);
  return 2;
}

int l_obj_getinfo(lua_State* L) {
  auto* ctx        = get_ctx(L);
  std::string key  = luaL_checkstring(L, 1);
  Image* img       = ctx->fpip->img;
  Composition* cmp = ctx->fpip->compo;
  if(key == "image_w") {
    lua_pushinteger(L, img ? img->width : 0);
    return 1;
  }
  if(key == "image_h") {
    lua_pushinteger(L, img ? img->height : 0);
    return 1;
  }
  if(key == "screen_w") {
    lua_pushinteger(L, cmp ? (int)cmp->size[0] : 0);
    return 1;
  }
  if(key == "screen_h") {
    lua_pushinteger(L, cmp ? (int)cmp->size[1] : 0);
    return 1;
  }
  if(key == "framerate") {
    lua_pushnumber(L, cmp ? cmp->framerate : 30.0);
    return 1;
  }
  lua_pushnil(L);
  return 1;
}

FilterPluginTable* find_filter_by_name(const char* name) {
  for(auto& plg : AppMain::Get()->filters)
    if(std::string(plg.name.c_str()) == name) return &plg;
  return nullptr;
}

// AviUtl内蔵エフェクトとmovutl内蔵フィルタは日本語名が一致するものが多いため、名前一致検索のみ行う(見つからなければベストエフォートでスキップ)
int l_obj_effect(lua_State* L) {
  auto* ctx              = get_ctx(L);
  std::string name       = luaL_checkstring(L, 1);
  FilterPluginTable* plg = find_filter_by_name(name.c_str());
  if(!plg) {
    LOG_F(WARNING, "obj.effect: 内部フィルタ '%s' が見つかりません(スキップ)", name.c_str());
    return 0;
  }
  cutil::Prop p = plg->defaults;
  int nargs     = lua_gettop(L);
  for(int i = 2; i + 1 <= nargs; i += 2) {
    if(!lua_isstring(L, i)) continue; // "name"のようなAviUtl固有の非数値パラメータはスキップ
    std::string pname = lua_tostring(L, i);
    if(!lua_isnumber(L, i + 1)) continue;
    double pval = lua_tonumber(L, i + 1);
    if(p.contains(pname.c_str())) {
      try {
        p.set<float>(pname.c_str(), (float)pval);
      } catch(const std::exception& e) {
        LOG_F(WARNING, "obj.effect: パラメータ'%s'設定失敗: %s", pname.c_str(), e.what());
      }
    }
  }
  if(plg->fn_proc) plg->fn_proc(plg, ctx->fpip, p);
  ctx->drawn = true;
  return 0;
}

// 引数省略時はobjテーブルの現在値(スクリプトが直接書き換えた値)を使う(AviUtl仕様に合わせる)
double obj_field_or_arg(lua_State* L, int argi, const char* field, double def) {
  if(!lua_isnoneornil(L, argi)) return luaL_checknumber(L, argi);
  lua_getglobal(L, "obj");
  lua_getfield(L, -1, field);
  double v = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : def;
  lua_pop(L, 2);
  return v;
}

// Image::copytoのcenter引数はpmin相当(内部でwidth/2が加算される)なので、AviUtlの中心原点オフセットx,yをそのまま渡す
void perform_draw(AviUtlObjContext* ctx, double x, double y, double zoom, double alpha, double rz) {
  Image* img = ctx->fpip->img;
  if(!img || img->empty()) return;
  Image tmp(img->width, img->height);
  tmp.has_alpha = true;
  std::memcpy(tmp.data(), img->data(), img->size_in_bytes());
  img->fill_rgba(Vec4b(0, 0, 0, 0));
  tmp.copyto(img, Vec2d(x, y), (float)zoom, (float)rz, (float)alpha, Blend_Alpha);
  ctx->drawn = true;
}

// obj.draw(x,y,z,zoom,alpha,rx,ry,rz): 現在の描画済みバッファを中心原点で移動・拡縮・Z回転して描き直す(rx/ryの3D回転は非対応)
int l_obj_draw(lua_State* L) {
  auto* ctx    = get_ctx(L);
  double x     = obj_field_or_arg(L, 1, "ox", 0.0);
  double y     = obj_field_or_arg(L, 2, "oy", 0.0);
  double zoom  = obj_field_or_arg(L, 4, "zoom", 1.0);
  double alpha = obj_field_or_arg(L, 5, "alpha", 1.0);
  double rz    = obj_field_or_arg(L, 8, "rz", 0.0);
  perform_draw(ctx, x, y, zoom, alpha, rz);
  return 0;
}

// obj.drawpoly(x0,y0,z0, x1,y1,z1, x2,y2,z2, x3,y3,z3): 四隅(左上,右上,左下,右下)を個別移動させ射影変形して描き直す(UV引数は非対応)
int l_obj_drawpoly(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img || img->empty()) return 0;
  if(lua_gettop(L) < 12) {
    LOG_F(WARNING, "obj.drawpoly: 引数が不足しています(x0,y0,z0,...,x3,y3,z3の12個が必要)");
    return 0;
  }

  Vec2d corners[4];
  for(int i = 0; i < 4; i++) {
    double x   = luaL_checknumber(L, i * 3 + 1);
    double y   = luaL_checknumber(L, i * 3 + 2);
    corners[i] = Vec2d(img->width / 2.0 + x, img->height / 2.0 + y);
  }

  Image tmp(img->width, img->height);
  tmp.has_alpha = true;
  std::memcpy(tmp.data(), img->data(), img->size_in_bytes());
  img->fill_rgba(Vec4b(0, 0, 0, 0));
  tmp.drawpoly(img, corners, 1.0f, Blend_Alpha);
  ctx->drawn = true;
  return 0;
}

// obj.copybuffer(dst,src): "obj"⇔"tmp"/"cache:xxx"間で現在の画像バッファを退避・復元する(フィルタインスタンス単位で永続)
int l_obj_copybuffer(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img || !ctx->buffers) return 0;
  std::string dst = luaL_checkstring(L, 1);
  std::string src = luaL_checkstring(L, 2);

  if(dst == "obj") {
    auto it = ctx->buffers->find(src);
    if(it == ctx->buffers->end()) return 0;
    Image& buf = it->second;
    img->resize(buf.width, buf.height);
    img->has_alpha = buf.has_alpha;
    std::memcpy(img->data(), buf.data(), img->size_in_bytes());
    ctx->drawn = true;
  } else {
    Image& buf = (*ctx->buffers)[dst];
    buf.resize(img->width, img->height);
    buf.has_alpha = img->has_alpha;
    std::memcpy(buf.data(), img->data(), img->size_in_bytes());
  }
  return 0;
}

// obj.setoption("drawtarget","tempbuffer",w,h): 描画バッファを拡張する(既存内容は中央基準で保持)。他のオプションは非対応でno-op
int l_obj_setoption(lua_State* L) {
  auto* ctx       = get_ctx(L);
  std::string opt = luaL_checkstring(L, 1);
  if(opt != "drawtarget" || lua_gettop(L) < 4 || !lua_isstring(L, 2)) return 0;
  std::string mode = lua_tostring(L, 2);
  if(mode != "tempbuffer") return 0;

  Image* img = ctx->fpip->img;
  int w      = (int)luaL_checknumber(L, 3);
  int h      = (int)luaL_checknumber(L, 4);
  if(!img || w <= 0 || h <= 0) return 0;

  Image tmp(w, h);
  tmp.has_alpha = true;
  int ox        = (w - (int)img->width) / 2;
  int oy        = (h - (int)img->height) / 2;
  img->copyto(&tmp, Vec2d(ox, oy));
  img->resize(w, h);
  img->has_alpha = true;
  std::memcpy(img->data(), tmp.data(), img->size_in_bytes());
  return 0;
}

// obj.load(type, ...): "tempbuffer"/"obj"は現在のバッファをそのまま使うno-op、それ以外(画像/動画/図形/テキスト読み込み)は今回未対応で警告のみ
int l_obj_load(lua_State* L) {
  std::string type = lua_isstring(L, 1) ? lua_tostring(L, 1) : "";
  if(type != "tempbuffer" && type != "obj") LOG_F(WARNING, "obj.load: 未対応の読み込み種別 '%s' をスキップしました", type.c_str());
  return 0;
}

int l_obj_noop(lua_State*) { return 0; }

// AviUtl組み込みグローバル関数RGB(color): 0xRRGGBBのパック整数を(r,g,b)の3値に分解する
int l_global_RGB(lua_State* L) {
  lua_Integer c = luaL_checkinteger(L, 1);
  lua_pushinteger(L, (c >> 16) & 0xFF);
  lua_pushinteger(L, (c >> 8) & 0xFF);
  lua_pushinteger(L, c & 0xFF);
  return 3;
}

} // namespace

void perform_implicit_draw(lua_State* L, AviUtlObjContext* ctx) {
  // 空スタック位置(999)を指定してobj_field_or_argを常にobjテーブルの現在値読み取りモードで動かす
  double x     = obj_field_or_arg(L, 999, "ox", 0.0);
  double y     = obj_field_or_arg(L, 999, "oy", 0.0);
  double zoom  = obj_field_or_arg(L, 999, "zoom", 1.0);
  double alpha = obj_field_or_arg(L, 999, "alpha", 1.0);
  double rz    = obj_field_or_arg(L, 999, "rz", 0.0);
  perform_draw(ctx, x, y, zoom, alpha, rz);
}

void setup_obj_table(lua_State* L, AviUtlObjContext* ctx) {
  lua_newtable(L);

  auto reg_fn = [&](const char* name, lua_CFunction fn) {
    lua_pushlightuserdata(L, ctx);
    lua_pushcclosure(L, fn, 1);
    lua_setfield(L, -2, name);
  };
  reg_fn("getpixeldata", l_obj_getpixeldata);
  reg_fn("putpixeldata", l_obj_putpixeldata);
  reg_fn("getpixel", l_obj_getpixel);
  reg_fn("getinfo", l_obj_getinfo);
  reg_fn("effect", l_obj_effect);
  reg_fn("draw", l_obj_draw);
  reg_fn("drawpoly", l_obj_drawpoly);
  reg_fn("copybuffer", l_obj_copybuffer);
  reg_fn("setoption", l_obj_setoption);
  reg_fn("getoption", l_obj_noop);
  reg_fn("setanchor", l_obj_noop); // アンカーポイント編集UIは今回未対応(no-op、呼び出し自体はエラーにしない)
  reg_fn("setfont", l_obj_noop);   // テキスト描画のフォント設定は今回未対応(no-op)
  reg_fn("load", l_obj_load);

  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "ox");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "oy");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "oz");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "rx");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "ry");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "rz");
  lua_pushnumber(L, 1);
  lua_setfield(L, -2, "zoom");
  lua_pushnumber(L, 1);
  lua_setfield(L, -2, "alpha");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "cx");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "cy");

  int total = 0;
  if(ctx->fpip->entt) total = ctx->fpip->entt->trk.fend - ctx->fpip->entt->trk.fstart;
  double time_sec = 0.0;
  if(ctx->fpip->compo && ctx->fpip->compo->framerate > 0) time_sec = ctx->frame / (double)ctx->fpip->compo->framerate;
  lua_pushinteger(L, ctx->frame);
  lua_setfield(L, -2, "frame");
  lua_pushinteger(L, total);
  lua_setfield(L, -2, "totalframe");
  lua_pushnumber(L, time_sec);
  lua_setfield(L, -2, "time");
  lua_pushinteger(L, 0);
  lua_setfield(L, -2, "layer");
  lua_pushnumber(L, ctx->fpip->compo ? ctx->fpip->compo->framerate : 30.0);
  lua_setfield(L, -2, "framerate");

  for(int i = 0; i < 4; i++) {
    std::string tname = "track" + std::to_string(i);
    lua_getglobal(L, tname.c_str());
    lua_setfield(L, -2, tname.c_str());
    std::string cname = "check" + std::to_string(i);
    lua_getglobal(L, cname.c_str());
    lua_setfield(L, -2, cname.c_str());
  }

  lua_setglobal(L, "obj");
}

void setup_global_functions(lua_State* L) {
  lua_pushcfunction(L, l_global_RGB);
  lua_setglobal(L, "RGB");
}

} // namespace mu::detail
