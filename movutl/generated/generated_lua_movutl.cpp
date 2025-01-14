// Movutl autogenerated file(LuaIntfWriter)
#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
#include <LuaIntf/LuaIntf.h>
#include <lua.hpp>
#include <imgui.h>
#include <movutl/core/props.hpp>
#include <movutl/app/app.hpp>
#include <movutl/plugin/input.hpp>
#include <movutl/plugin/filter.hpp>
#include <movutl/plugin/plugin.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/core/anim.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/composition.hpp>
extern "C" {
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

namespace mu::detail { 

using namespace LuaIntf;
using namespace ImGui;

void generated_lua_binding_(lua_State* L) {
    LuaBinding(L).beginModule("movutl")
  .beginClass<Composition>("Composition")
    .addFunction("resize", &Composition::resize)
    .addFunction("str", &Composition::str)
    .addFunction("summary", &Composition::summary)
    .addStaticFunction("GetActiveComp", &Composition::GetActiveComp)
    .addVariable("guid", &Composition::guid) // uint32_t
    .addVariable("name", &Composition::name) // FixString
    .addVariable("flag", &Composition::flag) // Flag
    .addVariable("frame_final", &Composition::frame_final) // Ref<Image>
    .addVariable("frame_edit", &Composition::frame_edit) // Ref<Image>
    .addVariable("frame_temp", &Composition::frame_temp) // Ref<Image>
    .addVariable("framerate_nu", &Composition::framerate_nu) // int32_t
    .addVariable("framerate_de", &Composition::framerate_de) // int32_t
    .addVariable("fstart", &Composition::fstart) // int32_t
    .addVariable("fend", &Composition::fend) // int32_t
    .addVariable("frame", &Composition::frame) // int32_t
    .addVariable("audio_n", &Composition::audio_n) // int32_t
    .addVariable("audio_ch", &Composition::audio_ch) // int32_t
    .addVariable("layers", &Composition::layers) // std::vector<TrackLayer>
  .endClass()
  .beginClass<EntityInfo>("EntityInfo")
    .addFunction("str", &EntityInfo::str)
    .addVariable("flag", &EntityInfo::flag) // EntityType
    .addVariable("framerate", &EntityInfo::framerate) // float
    .addVariable("nframes", &EntityInfo::nframes) // uint32_t
    .addVariable("format", &EntityInfo::format) // ImageFormat
    .addVariable("width", &EntityInfo::width) // uint16_t
    .addVariable("height", &EntityInfo::height) // uint16_t
    .addVariable("audio_n", &EntityInfo::audio_n) // int32_t
    .addVariable("audio_format_size", &EntityInfo::audio_format_size) // int32_t
  .endClass()
  .beginClass<Image>("Image")
    .addFunction("dirty", &Image::dirty)
    .addFunction("data", &Image::data)
    .addFunction("set_rgb", &Image::set_rgb)
    .addFunction("set_rgba", &Image::set_rgba)
    .addFunction("size", &Image::size)
    .addFunction("size_in_bytes", &Image::size_in_bytes)
    .addFunction("reset", &Image::reset)
    .addFunction("fill", &Image::fill)
    .addFunction("channels", &Image::channels)
    .addFunction("rgba", &Image::rgba)
    .addFunction("imshow", &Image::imshow)
    .addFunction("getType", &Image::getType)
    .addVariable("fmt", &Image::fmt) // ImageFormat
    .addVariable("width", &Image::width) // unsigned int
    .addVariable("height", &Image::height) // unsigned int
    .addVariable("pos", &Image::pos) // Vec3
    .addVariable("scale", &Image::scale) // Vec2
    .addVariable("rotation", &Image::rotation) // float
    .addVariable("alpha", &Image::alpha) // float
    .addVariable("path", &Image::path) // std::string
    .addVariable("dirty_", &Image::dirty_) // int16_t
  .endClass()
  .beginClass<Movie>("Movie")
    .addStaticFunction("Create", &Movie::Create)
    .addFunction("load_file", &Movie::load_file)
    .addFunction("getType", &Movie::getType)
    .addFunction("getPropsInfo", &Movie::getPropsInfo)
    .addFunction("getProps", &Movie::getProps)
    .addFunction("setProps", &Movie::setProps)
    .addVariable("pos", &Movie::pos) // Vec3
    .addVariable("scale", &Movie::scale) // Vec2
    .addVariable("rotation", &Movie::rotation) // float
    .addVariable("start_frame_", &Movie::start_frame_) // int
    .addVariable("speed", &Movie::speed) // float
    .addVariable("alpha_", &Movie::alpha_) // uint8_t
    .addVariable("loop_", &Movie::loop_) // bool
    .addVariable("with_alpha_", &Movie::with_alpha_) // bool
    .addVariable("path_", &Movie::path_) // std::string
  .endClass()
  .beginClass<Project>("Project")
    .addStaticFunction("New", &Project::New)
    .addFunction("get_main_comp", &Project::get_main_comp)
    .addStaticFunction("GetActiveCompo", &Project::GetActiveCompo)
    .addStaticFunction("SetActiveCompo", &Project::SetActiveCompo)
    .addVariable("path", &Project::path) // std::string
    .addVariable("output_path", &Project::output_path) // std::string
    .addVariable("entities", &Project::entities) // std::vector<Ref<Entity> >
    .addVariable("compos_", &Project::compos_) // std::vector<Composition>
    .addVariable("main_comp_idx", &Project::main_comp_idx) // int
  .endClass()
  .beginClass<TextEntt>("TextEntt")
    .addStaticFunction("Create", &TextEntt::Create)
    .addFunction("getType", &TextEntt::getType)
    .addFunction("getPropsInfo", &TextEntt::getPropsInfo)
    .addFunction("getProps", &TextEntt::getProps)
    .addFunction("setProps", &TextEntt::setProps)
    .addVariable("pos_", &TextEntt::pos_) // Vec3
    .addVariable("scale_x_", &TextEntt::scale_x_) // float
    .addVariable("scale_y_", &TextEntt::scale_y_) // float
    .addVariable("rot_", &TextEntt::rot_) // float
    .addVariable("speed", &TextEntt::speed) // float
    .addVariable("alpha_", &TextEntt::alpha_) // uint8_t
    .addVariable("font", &TextEntt::font) // std::string
    .addVariable("text", &TextEntt::text) // std::string
    .addVariable("separate", &TextEntt::separate) // bool
  .endClass()
  .beginClass<TrackLayer>("TrackLayer")
    .addFunction("find_entt", &TrackLayer::find_entt)
    .addFunction("str", &TrackLayer::str)
    .addFunction("summary", &TrackLayer::summary)
    .addVariable("name", &TrackLayer::name) // FixString
    .addVariable("active", &TrackLayer::active) // bool
    .addVariable("entts", &TrackLayer::entts) // std::vector<Ref<Entity> >
  .endClass()
    .addFunction("add_new_audio_track", static_cast<bool(*)( const char *, const char *, int, int)>(&add_new_audio_track))
    .addFunction("add_new_track", static_cast<bool(*)( const char *, EntityType, int, int)>(&add_new_track))
    .addFunction("add_new_video_track", static_cast<Ref<Entity>(*)( const char *, const char *, int, int)>(&add_new_video_track))
    .addFunction("clear_selected_entts", static_cast<void(*)( )>(&clear_selected_entts))
    .addFunction("cv_waitkey", static_cast<void(*)( int)>(&cv_waitkey))
    .addFunction("get_compatible_plugin", static_cast<InputPluginTable *(*)( const char *, EntityType)>(&get_compatible_plugin))
    .addFunction("get_selected_entts", static_cast<std::vector<Ref<Entity> >(*)( )>(&get_selected_entts))
    .addFunction("init", static_cast<void(*)( )>(&init))
    .addFunction("new_project", static_cast<void(*)( )>(&new_project))
    .addFunction("open_project", static_cast<void(*)( const char *)>(&open_project))
    .addFunction("save_project", static_cast<void(*)( )>(&save_project))
    .addFunction("save_project_as", static_cast<void(*)( const char *)>(&save_project_as))
    .addFunction("select_entt", static_cast<void(*)( const Ref<Entity> &)>(&select_entt))
    .addFunction("select_entts", static_cast<void(*)( const std::vector<Ref<Entity> > &)>(&select_entts))
    .addFunction("should_terminate", static_cast<bool(*)( )>(&should_terminate))
    .addFunction("terminate", static_cast<void(*)( )>(&terminate))
    .addFunction("update", static_cast<void(*)( )>(&update))
   .endModule();
}
} // namespace mu::detail
