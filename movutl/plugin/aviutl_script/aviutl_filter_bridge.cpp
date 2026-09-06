#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/aviutl_script/aviutl_obj_binding.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <movutl/plugin/plugin.hpp>
#include <sstream>
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
  int body_ref = LUA_NOREF;                       // luaL_loadstringでコンパイル済みの関数をレジストリに保持し、毎フレームの再コンパイルを避ける
  std::unordered_map<std::string, Image> buffers; // obj.copybufferの退避先。フィルタインスタンス単位でフレームをまたいで保持する
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
    setup_global_functions(state->L);
  }
  lua_State* L = state->L;

  // obj.copybuffer("obj","tmp")でフレーム開始時点のバッファへ復元できるよう、処理前の内容を"tmp"へ毎フレーム退避しておく(AviUtl本体の暗黙仕様)
  if(fpip->img) {
    Image& tmp_buf = state->buffers["tmp"];
    tmp_buf.resize(fpip->img->width, fpip->img->height);
    tmp_buf.has_alpha = fpip->img->has_alpha;
    std::memcpy(tmp_buf.data(), fpip->img->data(), fpip->img->size_in_bytes());
  }

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

  AviUtlObjContext ctx{fpip, fpip->frame, &state->def, false, &state->buffers};
  setup_obj_table(L, &ctx);

  if(state->body_ref == LUA_NOREF) {
    if(luaL_loadstring(L, state->def.lua_body.c_str()) != 0) {
      LOG_F(ERROR, "AviUtlスクリプト構文エラー(%s): %s", state->def.name.c_str(), lua_tostring(L, -1));
      lua_pop(L, 1);
      return false;
    }
    state->body_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  }
  lua_rawgeti(L, LUA_REGISTRYINDEX, state->body_ref);
  if(lua_pcall(L, 0, 0, 0) != 0) {
    LOG_F(ERROR, "AviUtlスクリプト実行エラー(%s): %s", state->def.name.c_str(), lua_tostring(L, -1));
    lua_pop(L, 1);
    return false;
  }
  // AviUtl本体同様、明示的な描画操作が一度も無ければobjの現在値でdraw()相当を暗黙的に実行する
  if(!ctx.drawn) perform_implicit_draw(L, &ctx);
  return true;
}

FilterPluginTable build_table(const AviUtlScriptDef& def) {
  FilterPluginTable t{};
  // 名前+本文からハッシュ値を作り安定した非0 guidにする(常に0だと別フィルタと衝突しEntity::fromSaveProps復元時に誤ったプラグインへ結び付く)
  t.guid              = 0xA51E000000000000ULL | (std::hash<std::string>{}(def.name + "\x1f" + def.lua_body) & 0x0000FFFFFFFFFFFFULL);
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

bool register_aviutl_filter(AviUtlScriptDef def) {
  if(!def.dialog_code.empty()) def.lua_body = def.dialog_code + def.lua_body; // --dialog:の変数初期化コードを本体の前に結合しておく
  auto state              = std::make_unique<AviUtlFilterState>();
  FilterPluginTable table = build_table(def);
  state->def              = std::move(def);

  auto& filters = AppMain::Get()->filters;
  filters.push_back(table);
  FilterPluginTable* stored = &filters.back();
  state_registry()[stored]  = std::move(state);
  return true;
}

void register_aviutl_scripts_from_file(const std::filesystem::path& path) {
  std::ifstream ifs(path);
  if(!ifs) {
    LOG_F(ERROR, "register_aviutl_scripts: ファイルを開けません: %s", path.string().c_str());
    return;
  }
  std::ostringstream ss;
  ss << ifs.rdbuf();
  auto defs = parse_aviutl_script(ss.str());
  if(defs.empty()) return;
  std::string stem = path.stem().string();
  for(auto& def : defs) {
    if(def.name.empty()) def.name = stem; // `@名前`ブロックが無いファイルはファイル名を効果名とする単一スクリプト扱いになる(AviUtl仕様)
    register_aviutl_filter(std::move(def));
  }
}

} // namespace

void register_aviutl_scripts() {
  namespace fs = std::filesystem;
  for(const auto& dir : Config::Get()->aviutl_script_paths) {
    if(!fs::exists(dir) || !fs::is_directory(dir)) continue;
    for(const auto& entry : fs::recursive_directory_iterator(dir)) {
      if(!entry.is_regular_file() || entry.path().extension() != ".anm") continue;
      LOG_F(1, "Loading AviUtl script: %s", entry.path().string().c_str());
      register_aviutl_scripts_from_file(entry.path());
    }
  }
}

} // namespace mu::detail
