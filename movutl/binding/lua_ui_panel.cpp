#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}
#include <LuaIntf/LuaIntf.h>
#include <imgui.h>
#include <movutl/asset/custom_object.hpp>
#include <movutl/binding/lua_ui_panel.hpp>
#include <movutl/gui/gui.hpp>

namespace mu {
namespace {

// Luaのテーブル(update関数)をUIPanel::Updateへ委譲するアダプタ
struct LuaUIPanel final : UIPanel {
  LuaUIPanel(std::string title, LuaIntf::LuaRef def) : title_(std::move(title)), def_(std::move(def)) {}

  void Update() override {
    ImGui::Begin(title_.c_str());
    auto fn = def_.get<LuaIntf::LuaRef>("update", LuaIntf::LuaRef());
    if(fn.isFunction()) fn.call<void>(def_);
    ImGui::End();
  }

  std::string title_;
  LuaIntf::LuaRef def_;
};

void lua_register_window(const std::string& title, LuaIntf::LuaRef def) { GUIManager::Get()->panels.push_back(cutil::make_ref<LuaUIPanel>(title, def)); }

// LuaIntfはstd::vector<std::string>の戻り値を自動変換できないため、LuaRefのテーブルを手で組み立てる
LuaIntf::LuaRef lua_list_custom_objects(lua_State* L) {
  auto t  = LuaIntf::LuaRef::createTable(L);
  int idx = 1;
  for(const auto& entry : CustomObjectRegistry::Get()->list()) t.set(idx++, entry.name);
  return t;
}

} // namespace

namespace detail {
void bind_lua_ui_panel_api(lua_State* L) { LuaIntf::LuaBinding(L).beginModule("movutl").addFunction("register_window", &lua_register_window).addFunction("list_custom_objects", &lua_list_custom_objects).endModule(); }
} // namespace detail

} // namespace mu
