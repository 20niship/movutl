#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/aviutl_script/aviutl_obj_binding.hpp>
#include <mutex>
#include <string>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

namespace mu::detail {

namespace {

AviUtlObjContext* get_ctx(lua_State* L) { return static_cast<AviUtlObjContext*>(lua_touserdata(L, lua_upvalueindex(1))); }

// obj.x/y/z/layer/idなど「対象Entityから引く値」の取得口。Entityの変換(pos等)の持ち方が変わってもここだけ差し替えればよい
struct ObjEntityInfo {
  double x = 0, y = 0, z = 0; // ponytail: Entity種別ごとにpos/pos_と持ち方が異なるため未接続(0固定)。Transform統一後にここで実値を返す
  int layer       = 0;        // 0始まりのレイヤー番号(Compositionに属さない場合は0)
  uint64_t id     = 0;
  int frame       = 0; // オブジェクト先頭からの経過フレーム
  int total_frame = 0;
};

ObjEntityInfo query_entity_info(const AviUtlObjContext* ctx) {
  ObjEntityInfo info;
  Entity* e  = ctx->fpip->entt;
  info.frame = ctx->frame;
  if(!e) return info;
  info.id          = e->guid_;
  info.frame       = ctx->frame - e->fstart_;
  info.total_frame = e->fend_ - e->fstart_;
  if(Composition* cmp = ctx->fpip->compo) {
    std::lock_guard<std::mutex> lock(cmp->mtx); // レンダリング中はcomp->mtxを保持していない(Entity::mtxのみ)ためデッドロックしない
    for(size_t i = 0; i < cmp->layers.size(); i++)
      for(auto& o : cmp->layers[i].entts)
        if(o.get() == e) info.layer = (int)i;
  }
  return info;
}

// objテーブルのw/hを現在の描画バッファのサイズに同期する(バッファを作り直す関数の後に呼ぶ)
void sync_obj_size(lua_State* L, const Image* img) {
  lua_getglobal(L, "obj");
  lua_pushinteger(L, img ? img->width : 0);
  lua_setfield(L, -2, "w");
  lua_pushinteger(L, img ? img->height : 0);
  lua_setfield(L, -2, "h");
  lua_pop(L, 1);
}

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

// obj.getpixel(): (w,h)を返す。obj.getpixel(x,y[,"col"]): 0始まりの画素を(r,g,b,a)、"col"指定時は(0xRRGGBB,a)で返す。範囲外は全て0
int l_obj_getpixel(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(lua_gettop(L) < 2) {
    lua_pushinteger(L, img ? img->width : 0);
    lua_pushinteger(L, img ? img->height : 0);
    return 2;
  }
  int x       = (int)std::floor(luaL_checknumber(L, 1));
  int y       = (int)std::floor(luaL_checknumber(L, 2));
  bool col    = lua_isstring(L, 3) && std::string(lua_tostring(L, 3)) == "col";
  bool inside = img && x >= 0 && y >= 0 && x < (int)img->width && y < (int)img->height;
  Vec4b c     = inside ? (*img)(x, y) : Vec4b(0, 0, 0, 0);
  if(col) {
    lua_pushinteger(L, (c[0] << 16) | (c[1] << 8) | c[2]);
    lua_pushinteger(L, c[3]);
    return 2;
  }
  for(int i = 0; i < 4; i++) lua_pushinteger(L, c[i]);
  return 4;
}

// obj.putpixel(x,y,r,g,b[,a]): 0始まりの画素を書き換える(範囲外は無視、aの既定は255)。putpixeldata同様、暗黙drawの対象外にする
int l_obj_putpixel(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img) return 0;
  int x = (int)std::floor(luaL_checknumber(L, 1));
  int y = (int)std::floor(luaL_checknumber(L, 2));
  if(x < 0 || y < 0 || x >= (int)img->width || y >= (int)img->height) return 0;
  auto ch      = [&](int i, double def) { return (uint8_t)std::clamp(luaL_optnumber(L, i, def), 0.0, 255.0); };
  (*img)(x, y) = Vec4b(ch(3, 0), ch(4, 0), ch(5, 0), ch(6, 255));
  ctx->drawn   = true;
  return 0;
}

// obj.copypixel(dx,dy,sx,sy): (sx,sy)の画素を(dx,dy)へコピーする(どちらかが範囲外なら何もしない)
int l_obj_copypixel(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img) return 0;
  int dx  = (int)std::floor(luaL_checknumber(L, 1));
  int dy  = (int)std::floor(luaL_checknumber(L, 2));
  int sx  = (int)std::floor(luaL_checknumber(L, 3));
  int sy  = (int)std::floor(luaL_checknumber(L, 4));
  auto in = [&](int x, int y) { return x >= 0 && y >= 0 && x < (int)img->width && y < (int)img->height; };
  if(!in(dx, dy) || !in(sx, sy)) return 0;
  (*img)(dx, dy) = (*img)(sx, sy);
  ctx->drawn     = true;
  return 0;
}

uint64_t splitmix64(uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

// obj.rand(min,max[,seed,frame]): [min,max]の整数を返す決定的乱数。seed指定時は(seed,frame)だけで値が決まり、省略時は(オブジェクトID,フレーム,呼び出し順)で決まる
int l_obj_rand(lua_State* L) {
  auto* ctx     = get_ctx(L);
  lua_Integer a = (lua_Integer)luaL_checknumber(L, 1);
  lua_Integer b = (lua_Integer)luaL_checknumber(L, 2);
  if(a > b) std::swap(a, b);
  ObjEntityInfo info = query_entity_info(ctx);
  bool has_seed      = !lua_isnoneornil(L, 3);
  uint64_t seed      = has_seed ? (uint64_t)(int64_t)luaL_checknumber(L, 3) : info.id;
  uint64_t frame     = (uint64_t)(int64_t)luaL_optnumber(L, 4, info.frame);
  uint64_t key       = splitmix64(seed) ^ splitmix64(frame + 0x1234567ULL);
  if(!has_seed) key = splitmix64(key + (uint64_t)ctx->rand_counter++);
  lua_pushinteger(L, a + (lua_Integer)(splitmix64(key) % (uint64_t)(b - a + 1)));
  return 1;
}

// obj.interpolation(time,x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3): 4点のCatmull-Romスプラインで、p1→p2間をtime(0-1)で補間した(x,y,z)を返す
int l_obj_interpolation(lua_State* L) {
  double t = luaL_checknumber(L, 1);
  double p[4][3];
  for(int i = 0; i < 4; i++)
    for(int k = 0; k < 3; k++) p[i][k] = luaL_checknumber(L, 2 + i * 3 + k);
  double t2 = t * t, t3 = t2 * t;
  for(int k = 0; k < 3; k++) lua_pushnumber(L, 0.5 * ((2 * p[1][k]) + (-p[0][k] + p[2][k]) * t + (2 * p[0][k] - 5 * p[1][k] + 4 * p[2][k] - p[3][k]) * t2 + (-p[0][k] + 3 * p[1][k] - 3 * p[2][k] + p[3][k]) * t3));
  return 3;
}

// obj.getvalue(target[,time]): 現在のobj変数("x","ox","zoom","rz"等)またはトラックバー("track0"-"track3")の値を返す。未知のtargetはnil
// ponytail: 他フレームの値(time指定)は保持していないので無視して現在値を返す
int l_obj_getvalue(lua_State* L) {
  std::string key = luaL_checkstring(L, 1);
  lua_getglobal(L, "obj");
  lua_getfield(L, -1, key.c_str());
  return lua_isnumber(L, -1) ? 1 : (lua_pushnil(L), 1);
}

// AviUtl正規のキーのみ対応。未対応キーはnilを返す(旧独自キーimage_w/image_h/screen_w/screen_h/framerateはobj.w/h/screen_w/screen_h/framerate変数へ移行済み)
// ponytail: saving/editing/multi_object/camera_modeはmovutlに対応する状態が無いので固定値。versionはAviUtl 1.10相当の値
int l_obj_getinfo(lua_State* L) {
  auto* ctx       = get_ctx(L);
  std::string key = luaL_checkstring(L, 1);
  if(key == "clock") {
    lua_pushnumber(L, std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
    return 1;
  }
  if(key == "saving" || key == "multi_object" || key == "camera_mode") {
    lua_pushboolean(L, 0);
    return 1;
  }
  if(key == "editing") {
    lua_pushboolean(L, 1);
    return 1;
  }
  if(key == "script_path") {
    lua_pushstring(L, "");
    return 1;
  }
  if(key == "version") {
    lua_pushinteger(L, 11000);
    return 1;
  }
  if(key == "image_max") {
    Composition* cmp = ctx->fpip->compo;
    lua_pushinteger(L, cmp ? (int)cmp->size[0] : 0);
    lua_pushinteger(L, cmp ? (int)cmp->size[1] : 0);
    return 2;
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
// cx,cy: 画像中心から見た基点(obj.cx/cy)。x,yは基点が置かれる位置なので、画像中心の位置は基点を回転・拡大した分だけずれる
void perform_draw(AviUtlObjContext* ctx, double x, double y, double zoom, double alpha, double rz, double cx, double cy) {
  Image* img = ctx->fpip->img;
  if(!img || img->empty()) return;
  const double rad = rz * M_PI / 180.0;
  x -= zoom * (cx * std::cos(rad) - cy * std::sin(rad));
  y -= zoom * (cx * std::sin(rad) + cy * std::cos(rad));
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
  perform_draw(ctx, x, y, zoom, alpha, rz, obj_field_or_arg(L, 999, "cx", 0.0), obj_field_or_arg(L, 999, "cy", 0.0));
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

// obj.line(x0,y0,x1,y1,r,g,b,a,width): AviUtl非標準の独自拡張。集中線等の放射状線画をピクセル直操作より高速に描くために追加(ponytail: 各ステップ正方形スタンプの素朴実装、アンチエイリアス無し)
int l_obj_line(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img || img->empty()) return 0;

  double x0 = luaL_checknumber(L, 1) + img->width / 2.0;
  double y0 = luaL_checknumber(L, 2) + img->height / 2.0;
  double x1 = luaL_checknumber(L, 3) + img->width / 2.0;
  double y1 = luaL_checknumber(L, 4) + img->height / 2.0;
  int r     = (int)luaL_checknumber(L, 5);
  int g     = (int)luaL_checknumber(L, 6);
  int b     = (int)luaL_checknumber(L, 7);
  int a     = (int)luaL_checknumber(L, 8);
  int width = (int)luaL_optnumber(L, 9, 1);
  if(width < 1) width = 1;

  double dx     = x1 - x0;
  double dy     = y1 - y0;
  double length = std::sqrt(dx * dx + dy * dy);
  int steps     = (int)std::ceil(length) + 1;
  int half      = width / 2;

  auto blend_px = [&](int px, int py) {
    if(px < 0 || py < 0 || px >= (int)img->width || py >= (int)img->height) return;
    Vec4b& dst = (*img)(px, py);
    float sa   = a / 255.0f;
    dst[0]     = (uint8_t)(r * sa + dst[0] * (1.0f - sa));
    dst[1]     = (uint8_t)(g * sa + dst[1] * (1.0f - sa));
    dst[2]     = (uint8_t)(b * sa + dst[2] * (1.0f - sa));
    dst[3]     = (uint8_t)std::min(255.0f, a + dst[3] * (1.0f - sa));
  };

  for(int s = 0; s <= steps; s++) {
    double t = steps == 0 ? 0.0 : (double)s / steps;
    int cx   = (int)std::round(x0 + dx * t);
    int cy   = (int)std::round(y0 + dy * t);
    for(int oy = -half; oy <= half; oy++)
      for(int ox = -half; ox <= half; ox++) blend_px(cx + ox, cy + oy);
  }
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
    sync_obj_size(L, img);
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
  sync_obj_size(L, img);
  return 0;
}

// obj.load("image",path): 画像ファイルで描画バッファを置き換える。"tempbuffer"/"obj"は現在のバッファをそのまま使うno-op。他(movie/figure/text等)は未対応で警告のみ
int l_obj_load(lua_State* L) {
  auto* ctx        = get_ctx(L);
  std::string type = lua_isstring(L, 1) ? lua_tostring(L, 1) : "";
  if(type == "image" && lua_isstring(L, 2) && ctx->fpip->img) {
    Image loaded;
    if(!loaded.load_file(lua_tostring(L, 2))) {
      LOG_F(WARNING, "obj.load: 画像を読み込めません: %s", lua_tostring(L, 2));
      return 0;
    }
    Image* img = ctx->fpip->img;
    img->resize(loaded.width, loaded.height);
    img->has_alpha = loaded.has_alpha;
    std::memcpy(img->data(), loaded.data(), img->size_in_bytes());
    sync_obj_size(L, img);
    return 0;
  }
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
  perform_draw(ctx, x, y, zoom, alpha, rz, obj_field_or_arg(L, 999, "cx", 0.0), obj_field_or_arg(L, 999, "cy", 0.0));
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
  reg_fn("putpixel", l_obj_putpixel);
  reg_fn("copypixel", l_obj_copypixel);
  reg_fn("rand", l_obj_rand);
  reg_fn("interpolation", l_obj_interpolation);
  reg_fn("getvalue", l_obj_getvalue);
  reg_fn("getinfo", l_obj_getinfo);
  reg_fn("effect", l_obj_effect);
  reg_fn("draw", l_obj_draw);
  reg_fn("drawpoly", l_obj_drawpoly);
  reg_fn("line", l_obj_line);
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
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "cz");
  lua_pushnumber(L, 0);
  lua_setfield(L, -2, "aspect");

  auto set_int = [&](const char* name, lua_Integer v) {
    lua_pushinteger(L, v);
    lua_setfield(L, -2, name);
  };
  auto set_num = [&](const char* name, double v) {
    lua_pushnumber(L, v);
    lua_setfield(L, -2, name);
  };

  ObjEntityInfo info = query_entity_info(ctx);
  double fps         = ctx->fpip->compo && ctx->fpip->compo->framerate > 0 ? ctx->fpip->compo->framerate : 30.0;
  const Image* img   = ctx->fpip->img;
  set_int("w", img ? img->width : 0);
  set_int("h", img ? img->height : 0);
  set_int("screen_w", ctx->fpip->compo ? (int)ctx->fpip->compo->size[0] : 0);
  set_int("screen_h", ctx->fpip->compo ? (int)ctx->fpip->compo->size[1] : 0);
  set_num("x", info.x);
  set_num("y", info.y);
  set_num("z", info.z);
  set_int("frame", info.frame);
  set_int("totalframe", info.total_frame);
  set_num("time", info.frame / fps);
  set_num("totaltime", info.total_frame / fps);
  set_int("layer", info.layer);
  set_int("index", 0); // 個別オブジェクト(テキストの文字毎など)は未対応のため常に単体扱い
  set_int("num", 1);
  set_int("id", (lua_Integer)info.id);
  set_num("framerate", fps);

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

namespace {
std::mutex g_snap_mtx;
std::string g_snap_name;
std::vector<std::pair<std::string, double>> g_snap_vars;
} // namespace

void store_obj_debug_snapshot(lua_State* L, const std::string& script_name) {
  static const char* keys[] = {"ox", "oy", "oz", "rx", "ry", "rz", "cx", "cy", "cz", "zoom", "aspect", "alpha", "x", "y", "z", "w", "h", "frame", "totalframe", "time", "layer", "index", "num", "id"};
  std::vector<std::pair<std::string, double>> vars;
  lua_getglobal(L, "obj");
  for(const char* k : keys) {
    lua_getfield(L, -1, k);
    if(lua_isnumber(L, -1)) vars.emplace_back(k, lua_tonumber(L, -1));
    lua_pop(L, 1);
  }
  lua_pop(L, 1);
  std::lock_guard<std::mutex> lock(g_snap_mtx);
  g_snap_name = script_name;
  g_snap_vars = std::move(vars);
}

std::pair<std::string, std::vector<std::pair<std::string, double>>> load_obj_debug_snapshot() {
  std::lock_guard<std::mutex> lock(g_snap_mtx);
  return {g_snap_name, g_snap_vars};
}

void setup_global_functions(lua_State* L) {
  lua_pushcfunction(L, l_global_RGB);
  lua_setglobal(L, "RGB");
}

} // namespace mu::detail
