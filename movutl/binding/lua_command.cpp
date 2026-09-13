#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}
#include <LuaIntf/LuaIntf.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/binding/lua_command.hpp>
#include <movutl/core/anim.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/filesystem.hpp>

namespace mu {
namespace {

CommandStatus lua_result_to_status(const LuaIntf::LuaRef& r) {
  if(!r.isValid()) return CommandStatus::Finished;
  if(r.type() == LuaIntf::LuaTypeID::STRING) {
    auto s = r.toValue<std::string>();
    if(s == "running") return CommandStatus::Running;
    if(s == "failed") return CommandStatus::Failed;
  } else if(r.type() == LuaIntf::LuaTypeID::BOOLEAN && !r.toValue<bool>()) {
    return CommandStatus::Failed;
  }
  return CommandStatus::Finished;
}

// Luaのテーブル(on_start/tick/on_undo/on_redo/undoable)をmCommandの仮想関数へ委譲するアダプタ
struct LuaCommand final : mCommand {
  explicit LuaCommand(LuaIntf::LuaRef def) : def_(std::move(def)) {}

  CommandStatus call_status_fn(const char* name) {
    auto fn = def_.get<LuaIntf::LuaRef>(name, LuaIntf::LuaRef());
    if(!fn.isFunction()) return CommandStatus::Finished;
    return lua_result_to_status(fn.call<LuaIntf::LuaRef>(def_));
  }
  void call_void_fn(const char* name) {
    auto fn = def_.get<LuaIntf::LuaRef>(name, LuaIntf::LuaRef());
    if(fn.isFunction()) fn.call<void>(def_);
  }

  CommandStatus on_start() override { return call_status_fn("on_start"); }
  CommandStatus tick() override { return call_status_fn("tick"); }
  void on_undo() override { call_void_fn("on_undo"); }
  void on_redo() override { call_void_fn("on_redo"); }
  bool undoable() const override { return def_.get<bool>("undoable", false); }

  LuaIntf::LuaRef def_;
};

void lua_register_command(const std::string& id, const std::string& name, const std::string& description, const std::string& shortcut, LuaIntf::LuaRef def) {
  register_command(CommandInfo{id, name, description, shortcut}, [def]() -> Ref<mCommand> { return cutil::make_ref<LuaCommand>(def); });
}

// LuaIntfはstd::vector<std::string>を自動変換できない(userdata扱いになる)ため、Luaの配列テーブルを手動で変換する
std::vector<std::string> lua_table_to_strings(const LuaIntf::LuaRef& t) {
  std::vector<std::string> out;
  if(!t.isTable()) return out;
  int n = t.len();
  for(int i = 1; i <= n; i++) out.push_back(t.get<std::string>(i));
  return out;
}

std::string lua_select_file_dialog(const std::string& title, const LuaIntf::LuaRef& extensions) { return select_file_dialog(title, lua_table_to_strings(extensions)); }

std::string lua_select_save_file_dialog(const std::string& title, const std::string& default_name, const LuaIntf::LuaRef& extensions) { return select_save_file_dialog(title, default_name, lua_table_to_strings(extensions)); }

// LuaIntfはmu::Vec2/Vec3/Vec4/Vec4bを自動変換できないため、1-indexed配列テーブルから手動で組み立てる
Vec2 lua_to_vec2(const LuaIntf::LuaRef& t) { return Vec2(t.get<float>(1), t.get<float>(2)); }
Vec3 lua_to_vec3(const LuaIntf::LuaRef& t) { return Vec3(t.get<float>(1), t.get<float>(2), t.get<float>(3)); }
Vec4 lua_to_vec4(const LuaIntf::LuaRef& t) { return Vec4(t.get<float>(1), t.get<float>(2), t.get<float>(3), t.get<float>(4)); }
Vec4b lua_to_vec4b(const LuaIntf::LuaRef& t) { return Vec4b((uint8_t)t.get<int>(1), (uint8_t)t.get<int>(2), (uint8_t)t.get<int>(3), (uint8_t)t.get<int>(4)); }

// inspector.cppの型switchと同じ並び。Entity*/未対応型はfalseを返す
bool lua_add_keyframe_to(AnimProps& anim, const std::string& prop, int frame, const LuaIntf::LuaRef& value) {
  int idx = anim.index_of(prop);
  if(idx < 0) return false;
  const cutil::PropInfo* t = anim.get_type(idx);
  uint32_t f               = (uint32_t)std::max(frame, 0);
  // clang-format off
  if(t == cutil::prop_info_of<float>()) return anim.add_keyframe<float>(idx, f, value.toValue<float>());
  if(t == cutil::prop_info_of<int32_t>()) return anim.add_keyframe<int>(idx, f, value.toValue<int>());
  if(t == cutil::prop_info_of<bool>()) return anim.add_keyframe<bool>(idx, f, value.toValue<bool>());
  if(t == cutil::prop_info_of<std::string>()) return anim.add_keyframe<std::string>(idx, f, value.toValue<std::string>());
  if(t == cutil::prop_info_of<Vec2>()) return anim.add_keyframe<Vec2>(idx, f, lua_to_vec2(value));
  if(t == cutil::prop_info_of<Vec3>()) return anim.add_keyframe<Vec3>(idx, f, lua_to_vec3(value));
  if(t == cutil::prop_info_of<Vec4>()) return anim.add_keyframe<Vec4>(idx, f, lua_to_vec4(value));
  if(t == cutil::prop_info_of<Vec4b>()) return anim.add_keyframe<Vec4b>(idx, f, lua_to_vec4b(value));
  // clang-format on
  return false;
}

void lua_invalidate_entity_cache(Entity* e) {
  if(auto* comp = e->get_comp()) comp->invalidate_cache_range(e->fstart_, e->fend_);
}

bool lua_add_keyframe_entity(Entity* e, const std::string& prop, int frame, LuaIntf::LuaRef value) {
  if(!e) return false;
  e->ensure_anim_props();
  bool ok = lua_add_keyframe_to(e->anim_props_, prop, frame, value);
  if(ok) lua_invalidate_entity_cache(e);
  return ok;
}

bool lua_remove_keyframe_entity(Entity* e, const std::string& prop, int frame) {
  if(!e) return false;
  e->ensure_anim_props();
  int idx = e->anim_props_.index_of(prop);
  if(idx < 0) return false;
  bool ok = e->anim_props_.erase_keyframe(idx, (uint32_t)std::max(frame, 0));
  if(ok) lua_invalidate_entity_cache(e);
  return ok;
}

bool lua_add_keyframe_filter(Entity* e, int filter_index, const std::string& prop, int frame, LuaIntf::LuaRef value) {
  if(!e || filter_index < 0 || filter_index >= (int)e->filters_.size()) return false;
  bool ok = lua_add_keyframe_to(e->filters_[filter_index].props, prop, frame, value);
  if(ok) lua_invalidate_entity_cache(e);
  return ok;
}

bool lua_remove_keyframe_filter(Entity* e, int filter_index, const std::string& prop, int frame) {
  if(!e || filter_index < 0 || filter_index >= (int)e->filters_.size()) return false;
  auto& props = e->filters_[filter_index].props;
  int idx     = props.index_of(prop);
  if(idx < 0) return false;
  bool ok = props.erase_keyframe(idx, (uint32_t)std::max(frame, 0));
  if(ok) lua_invalidate_entity_cache(e);
  return ok;
}

} // namespace

namespace detail {
void bind_lua_command_api(lua_State* L) {
  LuaIntf::LuaBinding(L)
    .beginModule("movutl")
    .addFunction("register_command", &lua_register_command)
    .addFunction("select_file_dialog", &lua_select_file_dialog)
    .addFunction("select_save_file_dialog", &lua_select_save_file_dialog)
    .addFunction("add_keyframe", &lua_add_keyframe_entity)
    .addFunction("remove_keyframe", &lua_remove_keyframe_entity)
    .addFunction("add_keyframe_filter", &lua_add_keyframe_filter)
    .addFunction("remove_keyframe_filter", &lua_remove_keyframe_filter)
    .endModule();
}
} // namespace detail

} // namespace mu
