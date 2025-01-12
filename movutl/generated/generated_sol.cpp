// Movutl autogenerated file(SolWriter)
#include <sol.hpp>
#include <sol/sol.hpp>
#include <sol/forward.hpp>
#include <lua.hpp>
namespace mu { 
void generated_lua_binding_pygen_(){
  sol::state_view lua;
lua.new_usertype<AnimKeyframe>("AnimKeyframe",
    sol::constructors<sol::types<>>(),
  "value_", &AnimKeyframe::value_,
  "frame_", &AnimKeyframe::frame_,
  "ease_", &AnimKeyframe::ease_,
  "ease2_", &AnimKeyframe::ease2_,
  "type", &AnimKeyframe::type,
);
lua.new_usertype<AnimProps>("AnimProps",
    sol::constructors<sol::types<>>(),
  "get", &AnimProps::get,
  "size", &AnimProps::size,
  "operator[]", &AnimProps::operator[],
  "operator[]", &AnimProps::operator[],
  "Types", &AnimProps::Types,
  "props", &AnimProps::props,
);
lua.new_usertype<Composition>("Composition",
    sol::constructors<sol::types<>>(),
  "resize", &Composition::resize,
  "Composition", &Composition::Composition,
  "Composition", &Composition::Composition,
  "str", &Composition::str,
  "summary", &Composition::summary,
  "guid", &Composition::guid,
  "name", &Composition::name,
  "flag", &Composition::flag,
  "frame_final", &Composition::frame_final,
  "frame_edit", &Composition::frame_edit,
  "frame_temp", &Composition::frame_temp,
  "framerate_nu", &Composition::framerate_nu,
  "framerate_de", &Composition::framerate_de,
  "fstart", &Composition::fstart,
  "fend", &Composition::fend,
  "frame", &Composition::frame,
  "audio_p", &Composition::audio_p,
  "audio_n", &Composition::audio_n,
  "audio_ch", &Composition::audio_ch,
  "layers", &Composition::layers,
);
lua.new_usertype<Entity>("Entity",
    sol::constructors<sol::types<>>(),
  "getType", &Entity::getType,
  "CreateEntity", &Entity::CreateEntity,
  "Find", &Entity::Find,
  "get_comp", &Entity::get_comp,
  "render", &Entity::render,
  "visible", &Entity::visible,
  "Entity", &Entity::Entity,
  "getPropsInfo", &Entity::getPropsInfo,
  "getProps", &Entity::getProps,
  "setProps", &Entity::setProps,
  "name", &Entity::name,
  "guid_", &Entity::guid_,
  "trk", &Entity::trk,
);
lua.new_usertype<EntityInfo>("EntityInfo",
    sol::constructors<sol::types<>>(),
  "str", &EntityInfo::str,
  "flag", &EntityInfo::flag,
  "framerate", &EntityInfo::framerate,
  "nframes", &EntityInfo::nframes,
  "format", &EntityInfo::format,
  "width", &EntityInfo::width,
  "height", &EntityInfo::height,
  "audio_n", &EntityInfo::audio_n,
  "audio_format", &EntityInfo::audio_format,
  "audio_format_size", &EntityInfo::audio_format_size,
  "handler", &EntityInfo::handler,
  "reserve", &EntityInfo::reserve,
);
lua.new_usertype<GUIManager>("GUIManager",
    sol::constructors<sol::types<>>(),
  "MOVUTL_DECLARE_SINGLETON", &GUIManager::MOVUTL_DECLARE_SINGLETON,
  "GUIManager", &GUIManager::GUIManager,
  "GUIManager", &GUIManager::GUIManager,
  "panels", &GUIManager::panels,
  "dockspace_id", &GUIManager::dockspace_id,
  "glfw_window", &GUIManager::glfw_window,
  "should_close", &GUIManager::should_close,
);
lua.new_usertype<Image>("Image",
    sol::constructors<sol::types<>>(),
  "Image", &Image::Image,
  "Image", &Image::Image,
  "set_rgb", &Image::set_rgb,
  "set_rgba", &Image::set_rgba,
  "copyto", &Image::copyto,
  "copyto", &Image::copyto,
  "copyto", &Image::copyto,
  "copyto", &Image::copyto,
  "data", &Image::data,
  "size", &Image::size,
  "size_in_bytes", &Image::size_in_bytes,
  "reset", &Image::reset,
  "fill", &Image::fill,
  "channels", &Image::channels,
  "resize", &Image::resize,
  "resize", &Image::resize,
  "at", &Image::at,
  "operator[]", &Image::operator[],
  "operator[]", &Image::operator[],
  "rgba", &Image::rgba,
  "operator()", &Image::operator(),
  "operator()", &Image::operator(),
  "set_cv_img", &Image::set_cv_img,
  "to_cv_img", &Image::to_cv_img,
  "imshow", &Image::imshow,
  "render", &Image::render,
  "getType", &Image::getType,
  "Create", &Image::Create,
  "Create", &Image::Create,
  "width", &Image::width,
  "height", &Image::height,
  "pos", &Image::pos,
  "scale_x", &Image::scale_x,
  "scale_y", &Image::scale_y,
  "rotation", &Image::rotation,
  "alpha", &Image::alpha,
  "path", &Image::path,
);
lua.new_usertype<Movie>("Movie",
    sol::constructors<sol::types<>>(),
  "Movie", &Movie::Movie,
  "Movie", &Movie::Movie,
  "Movie", &Movie::Movie,
  "Create", &Movie::Create,
  "load_file", &Movie::load_file,
  "getType", &Movie::getType,
  "render", &Movie::render,
  "getPropsInfo", &Movie::getPropsInfo,
  "getProps", &Movie::getProps,
  "setProps", &Movie::setProps,
  "start_frame_", &Movie::start_frame_,
  "speed", &Movie::speed,
  "alpha_", &Movie::alpha_,
  "loop_", &Movie::loop_,
  "with_alpha_", &Movie::with_alpha_,
  "path_", &Movie::path_,
);
lua.new_usertype<Project>("Project",
    sol::constructors<sol::types<>>(),
  "Project", &Project::Project,
  "Project", &Project::Project,
  "MOVUTL_DECLARE_SINGLETON", &Project::MOVUTL_DECLARE_SINGLETON,
  "New", &Project::New,
  "get_main_comp", &Project::get_main_comp,
  "GetActiveCompo", &Project::GetActiveCompo,
  "SetActiveCompo", &Project::SetActiveCompo,
  "path", &Project::path,
  "output_path", &Project::output_path,
  "entities", &Project::entities,
  "compos_", &Project::compos_,
  "main_comp_idx", &Project::main_comp_idx,
);
lua.new_usertype<PropAnimClip>("PropAnimClip",
    sol::constructors<sol::types<>>(),
  "PropAnimClip", &PropAnimClip::PropAnimClip,
  "reset", &PropAnimClip::reset,
  "get", &PropAnimClip::get,
  "add_keyframe", &PropAnimClip::add_keyframe,
  "has_animation", &PropAnimClip::has_animation,
  "clear", &PropAnimClip::clear,
  "keys", &PropAnimClip::keys,
  "keyname", &PropAnimClip::keyname,
);
lua.new_usertype<TextEntt>("TextEntt",
    sol::constructors<sol::types<>>(),
  "TextEntt", &TextEntt::TextEntt,
  "TextEntt", &TextEntt::TextEntt,
  "TextEntt", &TextEntt::TextEntt,
  "Create", &TextEntt::Create,
  "getType", &TextEntt::getType,
  "render", &TextEntt::render,
  "getPropsInfo", &TextEntt::getPropsInfo,
  "getProps", &TextEntt::getProps,
  "setProps", &TextEntt::setProps,
  "pos_", &TextEntt::pos_,
  "scale_x_", &TextEntt::scale_x_,
  "scale_y_", &TextEntt::scale_y_,
  "rot_", &TextEntt::rot_,
  "speed", &TextEntt::speed,
  "alpha_", &TextEntt::alpha_,
  "font", &TextEntt::font,
  "text", &TextEntt::text,
  "separate", &TextEntt::separate,
);
lua.new_usertype<TrackLayer>("TrackLayer",
    sol::constructors<sol::types<>>(),
  "find_entt", &TrackLayer::find_entt,
  "str", &TrackLayer::str,
  "summary", &TrackLayer::summary,
  "name", &TrackLayer::name,
  "active", &TrackLayer::active,
  "entts", &TrackLayer::entts,
);
lua.new_usertype<UIPanel>("UIPanel",
    sol::constructors<sol::types<>>(),
  "Update", &UIPanel::Update,
  "UIPanel", &UIPanel::UIPanel,
  "UIPanel", &UIPanel::UIPanel,
);
lua["MOVUTL_DEFINE_ENUM_ATTR_BITFLAGS"] = &MOVUTL_DEFINE_ENUM_ATTR_BITFLAGS;
lua["add_new_audio_track"] = &add_new_audio_track;
lua["add_new_track"] = &add_new_track;
lua["add_new_video_track"] = &add_new_video_track;
lua["clear_selected_entts"] = &clear_selected_entts;
lua["cv_waitkey"] = &cv_waitkey;
lua["get_compatible_plugin"] = &get_compatible_plugin;
lua["get_entt_icon"] = &get_entt_icon;
lua["get_selected_entts"] = &get_selected_entts;
lua["init"] = &init;
lua["new_project"] = &new_project;
lua["open_project"] = &open_project;
lua["save_project"] = &save_project;
lua["save_project_as"] = &save_project_as;
lua["select_entt"] = &select_entt;
lua["select_entts"] = &select_entts;
lua["should_terminate"] = &should_terminate;
lua["terminate"] = &terminate;
lua["update"] = &update;
# ---- end of file ----} // namespace mu