#include <algorithm>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu {

namespace {

// Composition座標(px) <-> Viewport画面座標 の変換。レターボックス配置+zoom/panを反映したimg_min/disp_sizeを渡す。
ImVec2 comp_to_screen(const ImVec2& p, const ImVec2& img_min, const ImVec2& disp_size, float cmp_w, float cmp_h) {
  return ImVec2(img_min.x + p.x / cmp_w * disp_size.x, img_min.y + p.y / cmp_h * disp_size.y);
}
ImVec2 screen_to_comp(const ImVec2& p, const ImVec2& img_min, const ImVec2& disp_size, float cmp_w, float cmp_h) {
  return ImVec2((p.x - img_min.x) / disp_size.x * cmp_w, (p.y - img_min.y) / disp_size.y * cmp_h);
}

} // namespace

void ViewerWindow::Update() {
  ImGui::Begin("Viewer");
  auto comp = Composition::GetActiveComp();
  if(!comp) {
    ImGui::Text("No active composition");
    ImGui::End();
    return;
  }

  if(!tex.initialized() && comp->frame_final) tex.set(comp->frame_final);
  tex.update_if_necessary();
  auto texture_id = tex.get_id();

  ImVec2 avail = ImGui::GetContentRegionAvail();
  if(avail.x < 1 || avail.y < 1) {
    ImGui::End();
    return;
  }
  ImVec2 origin = ImGui::GetCursorScreenPos();

  float cmp_w       = std::max(1.0f, (float)comp->size[0]);
  float cmp_h       = std::max(1.0f, (float)comp->size[1]);
  float base_scale  = std::min(avail.x / cmp_w, avail.y / cmp_h);
  ImVec2 base_size(cmp_w * base_scale, cmp_h * base_scale);
  ImVec2 base_pos(origin.x + (avail.x - base_size.x) / 2.0f, origin.y + (avail.y - base_size.y) / 2.0f);

  ImVec2 disp_size(base_size.x * zoom, base_size.y * zoom);
  ImVec2 center(base_pos.x + base_size.x / 2.0f + pan.x, base_pos.y + base_size.y / 2.0f + pan.y);
  ImVec2 img_min(center.x - disp_size.x / 2.0f, center.y - disp_size.y / 2.0f);
  ImVec2 img_max(img_min.x + disp_size.x, img_min.y + disp_size.y);

  ImGui::InvisibleButton("viewer_canvas", avail);
  bool hovered = ImGui::IsItemHovered();

  auto dl = ImGui::GetWindowDrawList();
  if(texture_id != 0) {
    ImTextureID tex_id = (ImTextureID) reinterpret_cast<void*>(static_cast<intptr_t>(texture_id));
    tex.bind();
    dl->AddImage(tex_id, img_min, img_max);
  }
  dl->AddRect(img_min, img_max, IM_COL32(255, 255, 255, 180)); // Composition境界線

  if(hovered) {
    if(ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
      auto d = ImGui::GetIO().MouseDelta;
      pan.x += d.x;
      pan.y += d.y;
    }

    float wheel = ImGui::GetIO().MouseWheel;
    if(wheel != 0.0f) {
      ImVec2 mouse   = ImGui::GetMousePos();
      ImVec2 comp_pt = screen_to_comp(mouse, img_min, disp_size, cmp_w, cmp_h);
      float new_zoom = std::clamp(zoom * (1.0f + wheel / 10.0f), 0.05f, 50.0f);
      ImVec2 new_disp_size(base_size.x * new_zoom, base_size.y * new_zoom);
      // マウス下のcomposition座標がズーム後も同じ画面位置に来るようpanを調整する
      ImVec2 new_img_min(mouse.x - comp_pt.x / cmp_w * new_disp_size.x, mouse.y - comp_pt.y / cmp_h * new_disp_size.y);
      pan.x = (new_img_min.x + new_disp_size.x / 2.0f) - (base_pos.x + base_size.x / 2.0f);
      pan.y = (new_img_min.y + new_disp_size.y / 2.0f) - (base_pos.y + base_size.y / 2.0f);
      zoom  = new_zoom;
    }

    if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
      zoom = 1.0f;
      pan  = ImVec2(0, 0);
    }
  }

  if(Config::Get()->show_viewer_ruler) {
    float px_per_x = disp_size.x / cmp_w;
    float px_per_y = disp_size.y / cmp_h;
    int di         = 100;
    if(px_per_x > 0) {
      float raw = 50.0f / px_per_x;
      if(raw < 10)
        di = 10;
      else if(raw < 50)
        di = 50;
      else if(raw < 100)
        di = 100;
      else if(raw < 200)
        di = 200;
      else
        di = 500;
    }
    for(int i = 0; i < (int)cmp_w; i += di) {
      auto p = comp_to_screen(ImVec2((float)i, 0), img_min, disp_size, cmp_w, cmp_h);
      dl->AddLine(ImVec2(p.x, img_min.y), ImVec2(p.x, img_min.y + 8), IM_COL32(255, 255, 0, 200));
      dl->AddText(ImVec2(p.x + 2, img_min.y), IM_COL32(255, 255, 0, 200), std::to_string(i).c_str());
    }
    for(int i = 0; i < (int)cmp_h; i += di) {
      auto p = comp_to_screen(ImVec2(0, (float)i), img_min, disp_size, cmp_w, cmp_h);
      dl->AddLine(ImVec2(img_min.x, p.y), ImVec2(img_min.x + 8, p.y), IM_COL32(255, 255, 0, 200));
      dl->AddText(ImVec2(img_min.x + 2, p.y), IM_COL32(255, 255, 0, 200), std::to_string(i).c_str());
    }
  }

  if(hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    ImVec2 comp_pt = screen_to_comp(ImGui::GetMousePos(), img_min, disp_size, cmp_w, cmp_h);
    Ref<Entity> hit;
    // TODO: anchor中心の近似矩形での簡易判定。Mesh側に汎用ジオメトリ取得が無いため回転/スケール後の正確なヒットテストは将来拡張
    constexpr float kHalfSize = 50.0f;
    for(auto& layer : comp->layers) {
      for(auto& e : layer.entts) {
        if(!e || !e->visible(comp->frame)) continue;
        ImRect r(ImVec2(e->trk.anchor[0] - kHalfSize, e->trk.anchor[1] - kHalfSize), ImVec2(e->trk.anchor[0] + kHalfSize, e->trk.anchor[1] + kHalfSize));
        if(r.Contains(ImVec2(comp_pt.x, comp_pt.y))) hit = e;
      }
    }
    if(hit) {
      clear_selected_entts();
      select_entt(hit);
    }
  }

  ImGui::End();
}
} // namespace mu
