#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// --
#include <movutl/app/app_impl.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/gui/composition_settings.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/inspector.hpp>
#include <movutl/gui/timeline.hpp>
#include <movutl/gui/timeline_window.hpp>
#include <movutl/gui/utilities.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu {
namespace detail {

void init_gui_panels() {
  auto g    = GUIManager::Get();
  g->panels = {
    cutil::make_ref<InspectorWindow>(), cutil::make_ref<TimelineWindow>(), cutil::make_ref<ViewerWindow>(), cutil::make_ref<UtilityWindow>(), cutil::make_ref<CompositionSettingsWindow>(),
  };

  // デフォルトワークスペース(初回起動時に適用される)
  Workspace default_workspace;
  default_workspace.name = "Default";
  default_workspace.add_entry("MOVUTL TIMELINE WINDOW", ImGuiDir_Down, 0.40f);
  default_workspace.add_entry("ツール", ImGuiDir_Left, 0.2f);
  default_workspace.add_entry(ICON_FA_PLUG " エフェクト制御", ImGuiDir_Right, 0.25f);
  default_workspace.add_entry("Viewer", ImGuiDir_None, 1.0f);
  register_workspace("Default", default_workspace);
}

void update_gui_panels() {
  MOVUTL_ZONE_SCOPED_N("update_gui_panels");
  auto a = GUIManager::Get();
  for(auto& panel : a->panels) panel->Update();
}

} // namespace detail

const char* get_entt_icon(const Ref<Entity>& entt) {
  if(!entt) return ICON_FA_QUESTION;
  switch(entt->getType()) {
    case EntityType::EntityType_Movie: return ICON_FA_VIDEO;
    case EntityType::EntityType_Audio: return ICON_FA_MUSIC;
    case EntityType::EntityType_Image: return ICON_FA_IMAGE;
    case EntityType::EntityType_3DText: return ICON_FA_FONT;
    case EntityType::EntityType_Primitive: return ICON_FA_CUBE;
    case EntityType::EntityType_Framebuffer: return ICON_FA_TV;
    case EntityType::EntityType_Polygon: return ICON_FA_DRAW_POLYGON;
    case EntityType::EntityType_Group: return ICON_FA_LAYER_GROUP;
    case EntityType::EntityType_Scene: return ICON_FA_GLOBE;
    case EntityType::EntityType_SceneAudio: return ICON_FA_GLOBE ICON_FA_MUSIC;
    case EntityType::EntityType_LayerCopy: return ICON_FA_COPY;
    case EntityType::EntityType_Particle: return ICON_FA_FIRE;
    case EntityType::EntityType_Custom: return ICON_FA_CIRCLE_NODES;
    case EntityType::EntityType_3DModel: return ICON_FA_CUBE;
    case EntityType::EntityType_Camera: return ICON_FA_VIDEO;
    case EntityType::EntityType_Effect: return ICON_FA_PLUG;
    default: return ICON_FA_QUESTION;
  }
  return ICON_FA_QUESTION;
}

ImU32 get_entt_color(const Ref<Entity>& entt) {
  if(!entt) return IM_COL32(150, 150, 150, 150);
  switch(entt->getType()) {
    case EntityType::EntityType_Movie: return IM_COL32(80, 120, 220, 150);
    case EntityType::EntityType_Audio: return IM_COL32(90, 200, 120, 150);
    case EntityType::EntityType_Image: return IM_COL32(220, 160, 60, 150);
    case EntityType::EntityType_3DText: return IM_COL32(220, 90, 200, 150);
    case EntityType::EntityType_Primitive: return IM_COL32(200, 200, 90, 150);
    case EntityType::EntityType_Framebuffer: return IM_COL32(90, 200, 200, 150);
    case EntityType::EntityType_Polygon: return IM_COL32(160, 120, 220, 150);
    case EntityType::EntityType_Group: return IM_COL32(150, 150, 150, 150);
    case EntityType::EntityType_Scene: return IM_COL32(60, 160, 220, 150);
    case EntityType::EntityType_SceneAudio: return IM_COL32(60, 200, 160, 150);
    case EntityType::EntityType_LayerCopy: return IM_COL32(180, 180, 180, 150);
    case EntityType::EntityType_Particle: return IM_COL32(230, 120, 60, 150);
    case EntityType::EntityType_Custom: return IM_COL32(140, 140, 220, 150);
    case EntityType::EntityType_3DModel: return IM_COL32(120, 220, 160, 150);
    case EntityType::EntityType_Camera: return IM_COL32(220, 220, 90, 150);
    case EntityType::EntityType_Effect: return IM_COL32(220, 90, 90, 150);
    default: return IM_COL32(150, 150, 150, 150);
  }
}

void register_imgui_style(const char* name, const ImGuiStyle& style) {
  auto app                = detail::AppMain::Get();
  app->imgui_styles[name] = style;
}

void apply_imgui_style(const char* name) {
  auto app = detail::AppMain::Get();
  if(!app->imgui_styles.contains(name)) {
    LOG_F(1, "apply_imgui_style: style %s not found", name);
    return;
  }
  LOG_F(1, "apply_imgui_style: %s", name);
  auto it           = app->imgui_styles[name];
  ImGui::GetStyle() = it;
}

void remove_imgui_style(const char* name) {
  auto app = detail::AppMain::Get();
  app->imgui_styles.erase(name);
}

} // namespace mu
