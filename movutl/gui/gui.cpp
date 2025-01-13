#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// --
#include <movutl/gui/gui.hpp>
#include <movutl/gui/inspector.hpp>
#include <movutl/gui/timeline.hpp>
#include <movutl/gui/timeline_window.hpp>
#include <movutl/gui/utilities.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu {
namespace detail {

void init_gui_panels() {
  auto g = GUIManager::Get();
  g->panels = {
    std::make_shared<InspectorWindow>(),
    std::make_shared<TimelineWindow>(),
    std::make_shared<ViewerWindow>(),
    std::make_shared<UtilityWindow>(),
  };
}

void update_gui_panels() {
  auto a = GUIManager::Get();
  for(auto& panel : a->panels) panel->Update();
  { // show fps in top panel transparent
    ImGui::SetNextWindowBgAlpha(0.35f);
    ImGui::SetNextWindowSize(ImVec2(100, 50));
    ImGui::Begin("FPS", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize);
    ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
    float ms = 1000.0f / ImGui::GetIO().Framerate;
    ImGui::Text("%.2f ms", ms);
    ImGui::End();
  }
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

} // namespace mu
