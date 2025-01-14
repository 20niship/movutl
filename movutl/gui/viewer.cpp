#include <imgui.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu {

void show_texture_with_imgui(mu::GLTexture* texture) {
  if(!texture->initialized()) {
    auto comp = Composition::GetActiveComp();
    if(!comp) return;
    auto img = comp->frame_final;
    if(!img) return;
    texture->set(img);
  }

  texture->update_if_necessary();

  // ImGuiで使用するためのテクスチャID
  auto texture_id = texture->get_id();
  if(texture_id == 0) {
    ImGui::Text("Invalid texture.");
    return;
  }

  int width = ImGui::GetWindowWidth();
  int height = ImGui::GetWindowHeight();

  ImTextureID tex_id = (ImTextureID) reinterpret_cast<void*>(static_cast<intptr_t>(texture_id));
  texture->bind();
  ImGui::Image(tex_id, ImVec2(width, height));
}

void ViewerWindow::Update() {
  ImGui::Begin("Viewer");
  show_texture_with_imgui(&tex);
  ImGui::End();
}
} // namespace mu
