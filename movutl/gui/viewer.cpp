#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/gui/audio_meter.hpp>
#include <movutl/gui/entity_gizmo.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/viewer.hpp>
#include <string>
#include <vector>

namespace mu {

namespace {

// Composition座標(px) <-> Viewport画面座標 の変換。レターボックス配置+zoom/panを反映したimg_min/disp_sizeを渡す。
ImVec2 comp_to_screen(const ImVec2& p, const ImVec2& img_min, const ImVec2& disp_size, float cmp_w, float cmp_h) { return ImVec2(img_min.x + p.x / cmp_w * disp_size.x, img_min.y + p.y / cmp_h * disp_size.y); }
ImVec2 screen_to_comp(const ImVec2& p, const ImVec2& img_min, const ImVec2& disp_size, float cmp_w, float cmp_h) { return ImVec2((p.x - img_min.x) / disp_size.x * cmp_w, (p.y - img_min.y) / disp_size.y * cmp_h); }

// HH:MM:SS:FF形式のタイムコード(タイムライン見出しと同じ表記)
std::string timecode_label(int frame, float fps) {
  const int fps_i = std::max(1, (int)std::round(fps > 0.0f ? fps : 30.0f));
  const int f     = std::max(0, frame);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d:%02d", f / fps_i / 3600, f / fps_i / 60 % 60, f / fps_i % 60, f % fps_i);
  return buf;
}

constexpr float kHandleHalf   = 4.0f;  // 角ハンドルの半サイズ(画面px)
constexpr float kRotHandleR   = 5.0f;  // 回転ハンドルの半径(画面px)
constexpr float kRotHandleGap = 24.0f; // 回転ハンドルを上辺から離す距離(画面px)
constexpr float kAnchorR      = 6.0f;  // 基点マーカーの半径(画面px)

// 選択Entityの変換ギズモ(枠・角ハンドル・回転ハンドル・基点マーカー)を描く
void draw_entity_gizmo(ImDrawList* dl, const EntityGizmo& g, const ImVec2& img_min, const ImVec2& disp_size, float cmp_w, float cmp_h) {
  auto to_screen  = [&](const GizmoPt& p) { return comp_to_screen(ImVec2((float)p.x, (float)p.y), img_min, disp_size, cmp_w, cmp_h); };
  const ImU32 col = IM_COL32(80, 170, 255, 255);
  ImVec2 pts[4];
  for(int i = 0; i < 4; i++) pts[i] = to_screen(g.quad.p[i]);
  dl->AddPolyline(pts, 4, col, ImDrawFlags_Closed, 1.5f);
  for(int i = 0; i < 4; i++) dl->AddRectFilled(ImVec2(pts[i].x - kHandleHalf, pts[i].y - kHandleHalf), ImVec2(pts[i].x + kHandleHalf, pts[i].y + kHandleHalf), IM_COL32(255, 255, 255, 255));
  const float scale_px = disp_size.x / cmp_w;
  const ImVec2 rot_h   = to_screen(gizmo_rotate_handle(g.quad, kRotHandleGap / scale_px));
  const ImVec2 top_mid((pts[0].x + pts[1].x) / 2, (pts[0].y + pts[1].y) / 2);
  dl->AddLine(top_mid, rot_h, col);
  dl->AddCircleFilled(rot_h, kRotHandleR, col);
  const ImVec2 ap = to_screen(g.anchor_pt);
  dl->AddCircle(ap, kAnchorR, IM_COL32(255, 200, 0, 255), 0, 1.5f);
  dl->AddLine(ImVec2(ap.x - kAnchorR - 3, ap.y), ImVec2(ap.x + kAnchorR + 3, ap.y), IM_COL32(255, 200, 0, 255));
  dl->AddLine(ImVec2(ap.x, ap.y - kAnchorR - 3), ImVec2(ap.x, ap.y + kAnchorR + 3), IM_COL32(255, 200, 0, 255));
}

} // namespace

ViewerCursor& viewer_cursor() {
  static ViewerCursor c;
  return c;
}

void ViewerWindow::Update() {
  MOVUTL_ZONE_SCOPED_N("ViewerWindow::Update");
  viewer_cursor().valid = false;
  ImGui::Begin("Viewer");
  auto comp = Composition::GetActiveComp();
  if(!comp) {
    ImGui::Text("No active composition");
    ImGui::End();
    return;
  }

  Ref<Image> img;
  if(comp->cache.get(comp->frame, &img)) {
    if(last_bound_frame_.lock().get() != img.get()) {
      tex.set(img);
      last_bound_frame_ = img;
    }
  } else if(!tex.initialized()) {
    img = comp->render_current_frame_main_thread(); // 初回のみ同期フォールバックで真っ黒を防ぐ
    if(img) {
      tex.set(img);
      last_bound_frame_ = img;
    }
  }
  // キャッシュ未ヒット時は直前のテクスチャをそのまま表示し続ける
  auto texture_id = tex.get_id();

  constexpr float kCtrlFooterH = 28.0f; // 操作行(再生/拡大率/波形)の高さ
  const float kFooterH         = kCtrlFooterH;
  ImVec2 avail                 = ImGui::GetContentRegionAvail();
  avail.y                      = std::max(1.0f, avail.y - kFooterH);
  if(avail.x < 1 || avail.y < 1) {
    ImGui::End();
    return;
  }
  ImVec2 origin   = ImGui::GetCursorScreenPos();
  auto reset_view = [&]() {
    zoom = 1.0f;
    pan  = ImVec2(0, 0);
  };

  float cmp_w      = std::max(1.0f, (float)comp->size[0]);
  float cmp_h      = std::max(1.0f, (float)comp->size[1]);
  float base_scale = std::min(avail.x / cmp_w, avail.y / cmp_h);
  ImVec2 base_size(cmp_w * base_scale, cmp_h * base_scale);
  ImVec2 base_pos(origin.x + (avail.x - base_size.x) / 2.0f, origin.y + (avail.y - base_size.y) / 2.0f);

  ImVec2 disp_size(base_size.x * zoom, base_size.y * zoom);
  ImVec2 center(base_pos.x + base_size.x / 2.0f + pan.x, base_pos.y + base_size.y / 2.0f + pan.y);
  ImVec2 img_min(center.x - disp_size.x / 2.0f, center.y - disp_size.y / 2.0f);
  ImVec2 img_max(img_min.x + disp_size.x, img_min.y + disp_size.y);

  ImGui::InvisibleButton("viewer_canvas", avail);
  bool hovered = ImGui::IsItemHovered();

  auto dl = ImGui::GetWindowDrawList();
  if(show_checker_) {
    const ImVec2 lo(std::max(img_min.x, origin.x), std::max(img_min.y, origin.y));
    const ImVec2 hi(std::min(img_max.x, origin.x + avail.x), std::min(img_max.y, origin.y + avail.y));
    constexpr float kCell = 12.0f;
    for(float y = lo.y; y < hi.y; y += kCell) {
      for(float x = lo.x; x < hi.x; x += kCell) {
        const bool odd = ((int)((x - img_min.x) / kCell) + (int)((y - img_min.y) / kCell)) & 1;
        dl->AddRectFilled(ImVec2(x, y), ImVec2(std::min(x + kCell, hi.x), std::min(y + kCell, hi.y)), odd ? IM_COL32(150, 150, 150, 255) : IM_COL32(100, 100, 100, 255));
      }
    }
  }
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

    if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) reset_view();
  }

  if(Config::Get()->show_viewer_ruler) {
    float px_per_x = disp_size.x / cmp_w;
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
    constexpr float kTick = 8.0f;  // 目盛り線の長さ
    constexpr float kGap  = 14.0f; // ラベル表示用にティックからさらに離す量
    const bool center     = Config::Get()->viewer_ruler_center_origin;
    for(auto& t : gizmo_ruler_ticks(cmp_w, di, center)) {
      auto p = comp_to_screen(ImVec2((float)t.comp_pos, 0), img_min, disp_size, cmp_w, cmp_h);
      dl->AddLine(ImVec2(p.x, img_min.y - kTick), ImVec2(p.x, img_min.y), IM_COL32(255, 255, 0, 200));
      dl->AddText(ImVec2(p.x + 2, img_min.y - kGap), IM_COL32(255, 255, 0, 200), std::to_string(t.value).c_str());
    }
    for(auto& t : gizmo_ruler_ticks(cmp_h, di, center)) {
      auto p = comp_to_screen(ImVec2(0, (float)t.comp_pos), img_min, disp_size, cmp_w, cmp_h);
      dl->AddLine(ImVec2(img_min.x - kTick, p.y), ImVec2(img_min.x, p.y), IM_COL32(255, 255, 0, 200));
      dl->AddText(ImVec2(img_min.x - kTick - 30.0f, p.y), IM_COL32(255, 255, 0, 200), std::to_string(t.value).c_str());
    }
  }

  if(show_grid_ || show_safe_ || show_center_) {
    dl->PushClipRect(ImVec2(std::max(img_min.x, origin.x), std::max(img_min.y, origin.y)), ImVec2(std::min(img_max.x, origin.x + avail.x), std::min(img_max.y, origin.y + avail.y)), true);
    auto line = [&](float x0, float y0, float x1, float y1, ImU32 col) { dl->AddLine(comp_to_screen(ImVec2(x0, y0), img_min, disp_size, cmp_w, cmp_h), comp_to_screen(ImVec2(x1, y1), img_min, disp_size, cmp_w, cmp_h), col); };
    if(show_grid_) {
      constexpr float kStep = 100.0f; // ponytail: 間隔はコンポpx固定(拡大率に応じた自動調整は未対応)
      for(float x = kStep; x < cmp_w; x += kStep) line(x, 0, x, cmp_h, IM_COL32(255, 255, 255, 50));
      for(float y = kStep; y < cmp_h; y += kStep) line(0, y, cmp_w, y, IM_COL32(255, 255, 255, 50));
    }
    if(show_safe_) {
      for(float m : {0.9f, 0.8f}) {
        const float x0 = cmp_w * (1 - m) / 2, y0 = cmp_h * (1 - m) / 2, x1 = cmp_w - x0, y1 = cmp_h - y0;
        const ImU32 col = m > 0.85f ? IM_COL32(255, 220, 80, 160) : IM_COL32(255, 140, 80, 160);
        line(x0, y0, x1, y0, col);
        line(x1, y0, x1, y1, col);
        line(x1, y1, x0, y1, col);
        line(x0, y1, x0, y0, col);
      }
    }
    if(show_center_) {
      line(cmp_w / 2, 0, cmp_w / 2, cmp_h, IM_COL32(80, 220, 255, 140));
      line(0, cmp_h / 2, cmp_w, cmp_h / 2, IM_COL32(80, 220, 255, 140));
    }
    dl->PopClipRect();
  }

  const GizmoPt comp_size{cmp_w, cmp_h};
  const float scale_px     = disp_size.x / cmp_w; // コンポ1pxあたりの画面px
  const GizmoPt mouse_comp = [&] {
    ImVec2 c = screen_to_comp(ImGui::GetMousePos(), img_min, disp_size, cmp_w, cmp_h);
    return GizmoPt{c.x, c.y};
  }();

  if(hovered && mouse_comp.x >= 0 && mouse_comp.y >= 0 && mouse_comp.x < cmp_w && mouse_comp.y < cmp_h) viewer_cursor() = {true, mouse_comp.x, mouse_comp.y};

  if(drag_.part != GizmoPart::None) {
    if(!ImGui::IsMouseDown(ImGuiMouseButton_Left) || !drag_.entt) {
      drag_ = {};
    } else {
      EntityGizmo g0;
      entity_gizmo_of(*drag_.entt, comp_size, g0); // src_size/origin_offsetの取得用(変換は開始時のs0を使う)
      const bool alt = ImGui::GetIO().KeyAlt;
      GizmoXform x   = drag_.s0;
      switch(drag_.part) {
        case GizmoPart::Body: x = gizmo_drag_move(drag_.s0, drag_.m0, mouse_comp); break;
        case GizmoPart::Scale: {
          const double hw = g0.src_size.x / 2, hh = g0.src_size.y / 2;
          const GizmoPt corners[4] = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
          x                        = gizmo_drag_scale(drag_.s0, g0.origin_offset, comp_size, corners[drag_.corner], mouse_comp);
          break;
        }
        case GizmoPart::Rotate: x = gizmo_drag_rotate(drag_.s0, comp_size, drag_.m0, mouse_comp); break;
        case GizmoPart::Anchor: {
          const GizmoPt local = gizmo_comp_to_local(drag_.s0, g0.origin_offset, comp_size, mouse_comp);
          x                   = gizmo_set_anchor(drag_.s0, local - g0.origin_offset, !alt); // Altで見た目の補正を無効化
          break;
        }
        default: break;
      }
      {
        std::lock_guard<std::mutex> lock(drag_.entt->mtx);
        entity_apply_xform(*drag_.entt, x);
      }
      comp->invalidate_cache_range(drag_.entt->fstart_, drag_.entt->fend_);
    }
  } else if(hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    GizmoHit hit;
    Ref<Entity> target;
    EntityGizmo g;
    // 選択中のEntityのハンドルを最優先(重なる他のEntityを選び直さない)
    for(auto& e : get_selected_entts()) {
      if(!e->visible(comp->frame) || !entity_gizmo_of(*e, comp_size, g)) continue;
      hit = gizmo_hit_test(g.quad, g.anchor_pt, mouse_comp, (kHandleHalf + 3.0f) / scale_px, kRotHandleGap / scale_px);
      if(hit.part != GizmoPart::None) {
        target = e;
        break;
      }
    }
    if(!target) {
      target = hit_test_entity(*comp, mouse_comp);
      if(target) {
        clear_selected_entts();
        select_entt(target);
        entity_gizmo_of(*target, comp_size, g);
        hit = {GizmoPart::Body, -1};
      }
    }
    if(target && hit.part != GizmoPart::None) drag_ = {hit.part, hit.corner, mouse_comp, g.xform, target};
  }

  {
    for(auto& e : get_selected_entts()) {
      EntityGizmo g;
      if(!e->visible(comp->frame) || !entity_gizmo_of(*e, comp_size, g)) continue;
      draw_entity_gizmo(dl, g, img_min, disp_size, cmp_w, cmp_h);
    }
  }

  {
    // 選択中のオブジェクトが見えない理由をビューア左上に表示する
    const char* hint = nullptr;
    int hidden = 0, offscreen = 0;
    for(auto& e : get_selected_entts()) {
      if(!e || !e->has_transform()) continue;
      EntityGizmo g;
      if(!e->visible(comp->frame)) {
        hidden++;
      } else if(entity_gizmo_of(*e, comp_size, g)) {
        bool any_in = false;
        for(int i = 0; i < 4; i++) any_in |= g.quad.p[i].x > 0 && g.quad.p[i].x < cmp_w && g.quad.p[i].y > 0 && g.quad.p[i].y < cmp_h;
        if(!any_in && !gizmo_point_in_quad(g.quad, GizmoPt{cmp_w / 2, cmp_h / 2})) offscreen++;
      }
    }
    if(hidden > 0)
      hint = "選択中のオブジェクトは現在のフレームでは表示されません";
    else if(offscreen > 0)
      hint = "選択中のオブジェクトは画面の外にあります";
    if(hint) {
      const ImVec2 ts = ImGui::CalcTextSize(hint);
      const ImVec2 p0(origin.x + 8, origin.y + 8);
      dl->AddRectFilled(p0, ImVec2(p0.x + ts.x + 12, p0.y + ts.y + 8), IM_COL32(0, 0, 0, 170), 4.0f);
      dl->AddText(ImVec2(p0.x + 6, p0.y + 4), IM_COL32(255, 210, 90, 255), hint);
    }
  }

  // フッター(1行): 再生操作 / タイムコード / 拡大率 / 表示補助 / 右側の残り幅に波形
  ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + avail.y));
  ImGui::BeginChild("##viewer_footer", ImVec2(0, kCtrlFooterH), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  if(ImGui::Button(ICON_FA_BACKWARD_STEP "##prev")) run_command("frame_step_backward");
  ImGui::SameLine(0, 2);
  if(ImGui::Button((std::string(is_playing() ? ICON_FA_PAUSE : ICON_FA_PLAY) + "##play").c_str())) run_command("play_pause");
  ImGui::SameLine(0, 2);
  if(ImGui::Button(ICON_FA_FORWARD_STEP "##next")) run_command("frame_step_forward");
  ImGui::SameLine();
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(timecode_label(comp->frame, comp->framerate).c_str());
  ImGui::SameLine();
  // 表示率は実寸(コンポ1px=画面1px)を100%とする
  float zoom_pct = zoom * base_scale * 100.0f;
  ImGui::SetNextItemWidth(72);
  if(ImGui::DragFloat("##zoom_pct", &zoom_pct, 1.0f, 5.0f, 5000.0f, "%.0f%%") && base_scale > 0.0f) zoom = zoom_pct / 100.0f / base_scale;
  ImGui::SameLine(0, 2);
  ImGui::SetNextItemWidth(ImGui::GetFrameHeight() + 4);
  if(ImGui::BeginCombo("##zoom_preset", "", ImGuiComboFlags_NoPreview)) {
    if(ImGui::Selectable(ICON_FA_EXPAND " フィット")) reset_view();
    for(int pct : {25, 50, 100, 200, 400}) {
      char label[16];
      std::snprintf(label, sizeof(label), "%d%%", pct);
      if(ImGui::Selectable(label) && base_scale > 0.0f) {
        zoom = pct / 100.0f / base_scale;
        pan  = ImVec2(0, 0);
      }
    }
    ImGui::EndCombo();
  }
  ImGui::SameLine(0, 2);
  if(ImGui::Button(ICON_FA_TABLE_CELLS "##view_opts")) ImGui::OpenPopup("##view_opts_popup");
  if(ImGui::IsItemHovered()) ImGui::SetTooltip("表示補助");
  if(ImGui::BeginPopup("##view_opts_popup")) {
    ImGui::Checkbox("透明を市松模様で表示", &show_checker_);
    ImGui::Checkbox("グリッド", &show_grid_);
    ImGui::Checkbox("セーフマージン", &show_safe_);
    ImGui::Checkbox("中心線", &show_center_);
    ImGui::EndPopup();
  }
  ImGui::SameLine();
  const ImVec2 wmin = ImGui::GetCursorScreenPos();
  const float wave_w = ImGui::GetContentRegionAvail().x;
  if(wave_w > 40.0f) draw_audio_wave(ImGui::GetWindowDrawList(), wmin, ImVec2(wmin.x + wave_w, wmin.y + ImGui::GetFrameHeight()), comp);
  ImGui::EndChild();

  ImGui::End();
}
} // namespace mu
