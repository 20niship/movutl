// Movutl autogenerated file(LuaIntfWriter)
#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
#include <LuaIntf/LuaIntf.h>
#include <lua.hpp>
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
namespace mu::detail { 

using namespace LuaIntf;

void generated_lua_binding_(lua_State* L) {
    auto module = LuaBinding(L).beginModule("movutl");
    module  .beginClass<Composition>("Composition")
    .addFunction("resize", &Composition::resize)
    .addFunction("str", &Composition::str)
    .addFunction("summary", &Composition::summary)
    .addStaticFunction("GetActiveComp", &Composition::GetActiveComp)
    .addVariable("guid", &Composition::guid)
    .addVariable("name", &Composition::name)
    .addVariable("flag", &Composition::flag)
    .addVariable("frame_final", &Composition::frame_final)
    .addVariable("frame_edit", &Composition::frame_edit)
    .addVariable("frame_temp", &Composition::frame_temp)
    .addVariable("framerate_nu", &Composition::framerate_nu)
    .addVariable("framerate_de", &Composition::framerate_de)
    .addVariable("fstart", &Composition::fstart)
    .addVariable("fend", &Composition::fend)
    .addVariable("frame", &Composition::frame)
    .addVariable("audio_p", &Composition::audio_p)
    .addVariable("audio_n", &Composition::audio_n)
    .addVariable("audio_ch", &Composition::audio_ch)
    .addVariable("layers", &Composition::layers)
  .endClass()
  .beginClass<EntityInfo>("EntityInfo")
    .addFunction("str", &EntityInfo::str)
    .addVariable("flag", &EntityInfo::flag)
    .addVariable("framerate", &EntityInfo::framerate)
    .addVariable("nframes", &EntityInfo::nframes)
    .addVariable("format", &EntityInfo::format)
    .addVariable("width", &EntityInfo::width)
    .addVariable("height", &EntityInfo::height)
    .addVariable("audio_n", &EntityInfo::audio_n)
    .addVariable("audio_format", &EntityInfo::audio_format)
    .addVariable("audio_format_size", &EntityInfo::audio_format_size)
    .addVariable("handler", &EntityInfo::handler)
    .addVariable("reserve", &EntityInfo::reserve)
  .endClass()
  .beginClass<Image>("Image")
    .addFunction("dirty", &Image::dirty)
    .addFunction("data", &Image::data)
    .addFunction("set_rgb", &Image::set_rgb)
    .addFunction("set_rgba", &Image::set_rgba)
    .addFunction("copyto_rgba", &Image::copyto_rgba)
    .addFunction("size", &Image::size)
    .addFunction("size_in_bytes", &Image::size_in_bytes)
    .addFunction("reset", &Image::reset)
    .addFunction("fill", &Image::fill)
    .addFunction("channels", &Image::channels)
    .addFunction("rgba", &Image::rgba)
    .addFunction("set_cv_img", &Image::set_cv_img)
    .addFunction("to_cv_img", &Image::to_cv_img)
    .addFunction("imshow", &Image::imshow)
    .addFunction("render", &Image::render)
    .addFunction("getType", &Image::getType)
    .addVariable("fmt", &Image::fmt)
    .addVariable("width", &Image::width)
    .addVariable("height", &Image::height)
    .addVariable("pos", &Image::pos)
    .addVariable("scale", &Image::scale)
    .addVariable("rotation", &Image::rotation)
    .addVariable("alpha", &Image::alpha)
    .addVariable("path", &Image::path)
    .addVariable("dirty_", &Image::dirty_)
  .endClass()
  .beginClass<Movie>("Movie")
    .addStaticFunction("Create", &Movie::Create)
    .addFunction("load_file", &Movie::load_file)
    .addFunction("getType", &Movie::getType)
    .addFunction("render", &Movie::render)
    .addFunction("getPropsInfo", &Movie::getPropsInfo)
    .addFunction("getProps", &Movie::getProps)
    .addFunction("setProps", &Movie::setProps)
    .addVariable("pos", &Movie::pos)
    .addVariable("scale", &Movie::scale)
    .addVariable("rotation", &Movie::rotation)
    .addVariable("start_frame_", &Movie::start_frame_)
    .addVariable("speed", &Movie::speed)
    .addVariable("alpha_", &Movie::alpha_)
    .addVariable("loop_", &Movie::loop_)
    .addVariable("with_alpha_", &Movie::with_alpha_)
    .addVariable("path_", &Movie::path_)
  .endClass()
  .beginClass<Project>("Project")
    .addStaticFunction("New", &Project::New)
    .addFunction("get_main_comp", &Project::get_main_comp)
    .addStaticFunction("GetActiveCompo", &Project::GetActiveCompo)
    .addStaticFunction("SetActiveCompo", &Project::SetActiveCompo)
    .addVariable("path", &Project::path)
    .addVariable("output_path", &Project::output_path)
    .addVariable("entities", &Project::entities)
    .addVariable("compos_", &Project::compos_)
    .addVariable("main_comp_idx", &Project::main_comp_idx)
  .endClass()
  .beginClass<TextEntt>("TextEntt")
    .addStaticFunction("Create", &TextEntt::Create)
    .addFunction("getType", &TextEntt::getType)
    .addFunction("render", &TextEntt::render)
    .addFunction("getPropsInfo", &TextEntt::getPropsInfo)
    .addFunction("getProps", &TextEntt::getProps)
    .addFunction("setProps", &TextEntt::setProps)
    .addVariable("pos_", &TextEntt::pos_)
    .addVariable("scale_x_", &TextEntt::scale_x_)
    .addVariable("scale_y_", &TextEntt::scale_y_)
    .addVariable("rot_", &TextEntt::rot_)
    .addVariable("speed", &TextEntt::speed)
    .addVariable("alpha_", &TextEntt::alpha_)
    .addVariable("font", &TextEntt::font)
    .addVariable("text", &TextEntt::text)
    .addVariable("separate", &TextEntt::separate)
  .endClass()
  .beginClass<TrackLayer>("TrackLayer")
    .addFunction("find_entt", &TrackLayer::find_entt)
    .addFunction("str", &TrackLayer::str)
    .addFunction("summary", &TrackLayer::summary)
    .addVariable("name", &TrackLayer::name)
    .addVariable("active", &TrackLayer::active)
    .addVariable("entts", &TrackLayer::entts)
  .endClass()
    .addFunction("add_new_audio_track", &add_new_audio_track)
    .addFunction("add_new_track", &add_new_track)
    .addFunction("add_new_video_track", &add_new_video_track)
    .addFunction("clear_selected_entts", &clear_selected_entts)
    .addFunction("cv_waitkey", &cv_waitkey)
    .addFunction("get_compatible_plugin", &get_compatible_plugin)
    .addFunction("get_selected_entts", &get_selected_entts)
    .addFunction("init", &init)
    .addFunction("new_project", &new_project)
    .addFunction("open_project", &open_project)
    .addFunction("save_project", &save_project)
    .addFunction("save_project_as", &save_project_as)
    .addFunction("select_entt", &select_entt)
    .addFunction("select_entts", &select_entts)
    .addFunction("should_terminate", &should_terminate)
    .addFunction("terminate", &terminate)
    .addFunction("update", &update)
   .endModule();
}
} // namespace mu::detail
