#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}
#include <LuaIntf/LuaIntf.h>
#include <movutl/binding/lua_command.hpp>
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

} // namespace

namespace detail {
void bind_lua_command_api(lua_State* L) { LuaIntf::LuaBinding(L).beginModule("movutl").addFunction("register_command", &lua_register_command).addFunction("select_file_dialog", &select_file_dialog).addFunction("select_save_file_dialog", &select_save_file_dialog).endModule(); }
} // namespace detail

} // namespace mu
