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

int l_obj_getpixeldata(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img) return 0;
  lua_pushlstring(L, reinterpret_cast<const char*>(img->data()), img->size_in_bytes());
  return 1;
}

int l_obj_putpixeldata(lua_State* L) {
  auto* ctx  = get_ctx(L);
  Image* img = ctx->fpip->img;
  if(!img) return 0;
  size_t len       = 0;
  const char* data = luaL_checklstring(L, 1, &len);
  std::memcpy(img->data(), data, std::min(len, img->size_in_bytes()));
  return 0;
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

// AviUtl内蔵エフェクト名 -> movutl内蔵フィルタ名のベストエフォート対応表(完全一致は保証しない)
FilterPluginTable* find_filter_by_name(const char* name) {
  for(auto& plg : AppMain::Get()->filters)
    if(std::string(plg.name.c_str()) == name) return &plg;
  return nullptr;
}

const char* map_aviutl_effect_name(const std::string& name) {
  static const std::unordered_map<std::string, const char*> table = {
    {"ぼかし", "ぼかし"}, {"モザイク", "モザイク"}, {"単色化", "単色化"}, {"色調補正", "色調補正"}, {"グロー", "グロー"}, {"発光", "発光"}, {"エッジ抽出", "エッジ抽出"}, {"ノイズ", "フィルムグレイン"},
  };
  auto it = table.find(name);
  return it == table.end() ? nullptr : it->second;
}

int l_obj_effect(lua_State* L) {
  auto* ctx          = get_ctx(L);
  std::string name   = luaL_checkstring(L, 1);
  const char* mapped = map_aviutl_effect_name(name);
  if(!mapped) {
    LOG_F(WARNING, "obj.effect: 未対応のエフェクト名 '%s' をスキップしました", name.c_str());
    return 0;
  }
  FilterPluginTable* plg = find_filter_by_name(mapped);
  if(!plg) {
    LOG_F(WARNING, "obj.effect: 内部フィルタ '%s' が見つかりません", mapped);
    return 0;
  }
  cutil::Prop p = plg->defaults;
  int nargs     = lua_gettop(L);
  for(int i = 2; i + 1 <= nargs; i += 2) {
    std::string pname = luaL_checkstring(L, i);
    double pval       = luaL_checknumber(L, i + 1);
    if(p.contains(pname.c_str())) {
      try {
        p.set<float>(pname.c_str(), (float)pval);
      } catch(const std::exception& e) {
        LOG_F(WARNING, "obj.effect: パラメータ'%s'設定失敗: %s", pname.c_str(), e.what());
      }
    }
  }
  if(plg->fn_proc) plg->fn_proc(plg, ctx->fpip, p);
  return 0;
}

// obj.draw()単体呼び出しの受け皿(既に描画済みの画像を加工するアニメーション効果では実質no-op)
int l_obj_draw_noop(lua_State*) { return 0; }

} // namespace

void setup_obj_table(lua_State* L, AviUtlObjContext* ctx) {
  lua_newtable(L);

  auto reg_fn = [&](const char* name, lua_CFunction fn) {
    lua_pushlightuserdata(L, ctx);
    lua_pushcclosure(L, fn, 1);
    lua_setfield(L, -2, name);
  };
  reg_fn("getpixeldata", l_obj_getpixeldata);
  reg_fn("putpixeldata", l_obj_putpixeldata);
  reg_fn("getinfo", l_obj_getinfo);
  reg_fn("effect", l_obj_effect);
  reg_fn("draw", l_obj_draw_noop);
  reg_fn("drawpoly", l_obj_draw_noop);
  reg_fn("copybuffer", l_obj_draw_noop);
  reg_fn("setoption", l_obj_draw_noop);
  reg_fn("getoption", l_obj_draw_noop);

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

} // namespace mu::detail
