#include <cstring>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/prop_types.hpp>
#include <movutl/plugin/aviutl_script/aviutl_obj_binding.hpp>
#include <movutl/plugin/filter.hpp>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

namespace mu {

CustomObjectEntt::~CustomObjectEntt() {
  if(L_) lua_close(L_);
}

Ref<CustomObjectEntt> CustomObjectEntt::Create(const char* name, const std::string& script_name) {
  const auto* def = CustomObjectRegistry::Get()->find_def(script_name);
  if(!def) {
    LOG_F(ERROR, "CustomObjectEntt::Create: カスタムオブジェクト '%s' が見つかりません", script_name.c_str());
    return nullptr;
  }
  auto e          = cutil::make_ref<CustomObjectEntt>();
  e->def_         = def;
  e->script_name_ = script_name;
  e->name         = name;
  for(auto& tr : def->tracks) e->params_.set<float>(tr.name.c_str(), tr.default_value);
  for(auto& ch : def->checks) e->params_.set<bool>(ch.name.c_str(), ch.default_value);
  Project::Get()->entities.push_back(e);
  e->guid_ = Project::Get()->entities.size();
  return e;
}

bool CustomObjectEntt::render(Composition* cmp, Image* target, int frame) {
  if(!def_) return false;
  MU_ASSERT(cmp != nullptr && target != nullptr);
  target->resize((int)cmp->size[0], (int)cmp->size[1]);
  target->has_alpha = true;
  target->fill_rgba(Vec4b(0, 0, 0, 0));

  if(!L_) {
    L_ = luaL_newstate();
    luaL_openlibs(L_);
    detail::setup_global_functions(L_);
  }

  for(size_t i = 0; i < def_->tracks.size() && i < 4; i++) {
    float v = cutil::get_or<float>(params_, def_->tracks[i].name.c_str(), def_->tracks[i].default_value);
    lua_pushnumber(L_, v);
    lua_setglobal(L_, ("track" + std::to_string(i)).c_str());
  }
  for(size_t i = 0; i < def_->checks.size() && i < 4; i++) {
    bool v = cutil::get_or<bool>(params_, def_->checks[i].name.c_str(), def_->checks[i].default_value);
    lua_pushboolean(L_, v);
    lua_setglobal(L_, ("check" + std::to_string(i)).c_str());
  }

  FilterInData fpip;
  fpip.img   = target;
  fpip.compo = cmp;
  fpip.entt  = this;
  fpip.frame = frame;

  detail::AviUtlObjContext ctx{&fpip, frame, def_, false, &buffers_};
  detail::setup_obj_table(L_, &ctx);

  if(body_ref_ == LUA_NOREF) {
    if(luaL_loadstring(L_, def_->lua_body.c_str()) != 0) {
      LOG_F(ERROR, "カスタムオブジェクト構文エラー(%s): %s", def_->name.c_str(), lua_tostring(L_, -1));
      lua_pop(L_, 1);
      return false;
    }
    body_ref_ = luaL_ref(L_, LUA_REGISTRYINDEX);
  }
  lua_rawgeti(L_, LUA_REGISTRYINDEX, body_ref_);
  if(lua_pcall(L_, 0, 0, 0) != 0) {
    LOG_F(ERROR, "カスタムオブジェクト実行エラー(%s): %s", def_->name.c_str(), lua_tostring(L_, -1));
    lua_pop(L_, 1);
    return false;
  }
  if(!ctx.drawn) detail::perform_implicit_draw(L_, &ctx);
  return true;
}

cutil::Prop CustomObjectEntt::getProps() const {
  cutil::Prop p;
  p.set<std::string>("script_name", script_name_);
  p.set_child("params", params_);
  return p;
}

void CustomObjectEntt::setProps(const cutil::Prop& props) {
  script_name_ = cutil::get_or<std::string>(props, "script_name", "");
  def_         = CustomObjectRegistry::Get()->find_def(script_name_);
  if(!def_) LOG_F(WARNING, "CustomObjectEntt::setProps: カスタムオブジェクト '%s' が見つかりません(スクリプトが削除された可能性)", script_name_.c_str());
  if(props.contains("params")) params_ = props.get_child("params");
}

CustomObjectRegistry* CustomObjectRegistry::singleton_ = nullptr;

void CustomObjectRegistry::register_object(std::unique_ptr<detail::AviUtlScriptDef> def) {
  const detail::AviUtlScriptDef* raw = def.get();
  std::string name                   = def->name;
  defs_.push_back(std::move(def));
  entries_.push_back(Entry{name, [name]() -> Ref<Entity> { return CustomObjectEntt::Create(name.c_str(), name); }});
  (void)raw;
}

const detail::AviUtlScriptDef* CustomObjectRegistry::find_def(const std::string& name) const {
  for(auto& def : defs_)
    if(def->name == name) return def.get();
  return nullptr;
}

} // namespace mu
