#include <memory>
#include <movutl/app/app_impl.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/aviutl_script/aviutl_filter_bridge.hpp>
#include <movutl/plugin/aviutl_script/aviutl_obj_binding.hpp>
#include <unordered_map>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

namespace mu::detail {

namespace {

struct AviUtlFilterState {
  AviUtlScriptDef def;
  lua_State* L = nullptr;
  ~AviUtlFilterState() {
    if(L) lua_close(L);
  }
};

// fn_procのfp引数(=vector内FilterPluginTable*)をキーに状態を引く。vector<FilterPluginTable>は値保持のためreallocでアドレスが変わりうる点に注意(register_default_filters側でreserve済み)。
std::unordered_map<const FilterPluginTable*, std::unique_ptr<AviUtlFilterState>>& state_registry() {
  static std::unordered_map<const FilterPluginTable*, std::unique_ptr<AviUtlFilterState>> m;
  return m;
}

bool aviutl_fn_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  auto* table = static_cast<const FilterPluginTable*>(fp);
  auto it     = state_registry().find(table);
  if(it == state_registry().end()) {
    LOG_F(ERROR, "aviutl_fn_proc: スクリプト状態が見つかりません(%s)", table->name.c_str());
    return false;
  }
  AviUtlFilterState* state = it->second.get();
  if(!state->L) {
    state->L = luaL_newstate();
    luaL_openlibs(state->L);
  }
  lua_State* L = state->L;

  for(size_t i = 0; i < state->def.tracks.size() && i < 4; i++) {
    float v = cutil::get_or<float>(p, state->def.tracks[i].name.c_str(), state->def.tracks[i].default_value);
    lua_pushnumber(L, v);
    lua_setglobal(L, ("track" + std::to_string(i)).c_str());
  }
  for(size_t i = 0; i < state->def.checks.size() && i < 4; i++) {
    bool v = cutil::get_or<bool>(p, state->def.checks[i].name.c_str(), state->def.checks[i].default_value);
    lua_pushboolean(L, v);
    lua_setglobal(L, ("check" + std::to_string(i)).c_str());
  }

  AviUtlObjContext ctx{fpip, fpip->reserve[0], &state->def};
  setup_obj_table(L, &ctx);

  if(luaL_loadstring(L, state->def.lua_body.c_str()) != 0) {
    LOG_F(ERROR, "AviUtlスクリプト構文エラー(%s): %s", state->def.name.c_str(), lua_tostring(L, -1));
    lua_pop(L, 1);
    return false;
  }
  if(lua_pcall(L, 0, 0, 0) != 0) {
    LOG_F(ERROR, "AviUtlスクリプト実行エラー(%s): %s", state->def.name.c_str(), lua_tostring(L, -1));
    lua_pop(L, 1);
    return false;
  }
  return true;
}

FilterPluginTable build_table(const AviUtlScriptDef& def) {
  FilterPluginTable t{};
  t.guid              = 0;
  t.flag              = FilterDefault;
  t.name              = cutil::Str(def.name.c_str());
  t.info              = cutil::Str("AviUtl互換スクリプト");
  t.version           = 0;
  t.version_str       = "0";
  t.fn_cutstom_wnd    = nullptr;
  t.fn_update_value   = nullptr;
  t.fn_init           = nullptr;
  t.fn_exit           = nullptr;
  t.fn_proc           = aviutl_fn_proc;
  t.fn_update         = nullptr;
  t.func_is_saveframe = nullptr;
  t.fn_project_load   = nullptr;
  t.func_project_save = nullptr;

  for(auto& tr : def.tracks) {
    t.props.fields.push_back(cutil::PropInfo::Field(tr.name.c_str(), 0, cutil::prop_info_of<float>()));
    t.props.fields.back().set_label(tr.name.c_str());
    t.props.fields.back().min_value  = tr.min_value;
    t.props.fields.back().max_value  = tr.max_value;
    t.props.fields.back().drag_speed = tr.step;
    t.defaults.set<float>(tr.name.c_str(), tr.default_value);
  }
  for(auto& ch : def.checks) {
    t.props.fields.push_back(cutil::PropInfo::Field(ch.name.c_str(), 0, cutil::prop_info_of<bool>()));
    t.props.fields.back().set_label(ch.name.c_str());
    t.props.fields.back().widget = cutil::PropWidget::Checkbox;
    t.defaults.set<bool>(ch.name.c_str(), ch.default_value);
  }
  return t;
}

} // namespace

bool register_aviutl_filter(AviUtlScriptDef def) {
  auto state              = std::make_unique<AviUtlFilterState>();
  FilterPluginTable table = build_table(def);
  state->def              = std::move(def);

  auto& filters = AppMain::Get()->filters;
  filters.push_back(table);
  FilterPluginTable* stored = &filters.back();
  state_registry()[stored]  = std::move(state);
  return true;
}

} // namespace mu::detail
