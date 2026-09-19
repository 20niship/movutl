#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/gui/entity_gizmo.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/viewer.hpp>
#include <vector>

namespace mu {

namespace {

// Composition座標(px) <-> Viewport画面座標 の変換。レターボックス配置+zoom/panを反映したimg_min/disp_sizeを渡す。
ImVec2 comp_to_screen(const ImVec2& p, const ImVec2& img_min, const ImVec2& disp_size, float cmp_w, float cmp_h) { return ImVec2(img_min.x + p.x / cmp_w * disp_size.x, img_min.y + p.y / cmp_h * disp_size.y); }
ImVec2 screen_to_comp(const ImVec2& p, const ImVec2& img_min, const ImVec2& disp_size, float cmp_w, float cmp_h) { return ImVec2((p.x - img_min.x) / disp_size.x * cmp_w, (p.y - img_min.y) / disp_size.y * cmp_h); }

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

  const float kWaveFooterH     = Config::Get()->viewer_wave_footer_height; // 波形+L/Rメーターの高さ
  constexpr float kCtrlFooterH = 28.0f;                                    // ズーム率/フィットボタンの高さ
  const float kFooterH         = kWaveFooterH + kCtrlFooterH;
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
      const bool shift = ImGui::GetIO().KeyShift, alt = ImGui::GetIO().KeyAlt;
      GizmoXform x = drag_.s0;
      switch(drag_.part) {
        case GizmoPart::Body: x = gizmo_drag_move(drag_.s0, drag_.m0, mouse_comp); break;
        case GizmoPart::Scale: {
          const double hw = g0.src_size.x / 2, hh = g0.src_size.y / 2;
          const GizmoPt corners[4] = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
          x                        = gizmo_drag_scale(drag_.s0, g0.origin_offset, comp_size, corners[drag_.corner], mouse_comp, shift);
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

  // フッター1段目: 波形+L/Rメーター(ミックス後音声)
  if(comp->audio_buf) {
    int64_t cur_sample = comp->frame_to_sample(comp->frame);
    int channels       = std::max(1, comp->audio_channels);
    constexpr int kN   = 1024;
    std::vector<int16_t> pcm((size_t)kN * channels, 0);
    comp->audio_buf->snapshot(cur_sample, kN, pcm.data());

    ImVec2 footer_min = ImVec2(origin.x, origin.y + avail.y);
    ImVec2 footer_size(avail.x, kWaveFooterH);
    dl->AddRectFilled(footer_min, ImVec2(footer_min.x + footer_size.x, footer_min.y + footer_size.y), IM_COL32(20, 20, 20, 255));

    float meter_w = 30.0f;
    float wave_w  = std::max(1.0f, footer_size.x - meter_w);
    float mid_y   = footer_min.y + kWaveFooterH / 2.0f;
    for(int x = 0; x < (int)wave_w; x++) {
      int i     = x * kN / (int)wave_w;
      int16_t l = pcm[(size_t)i * channels];
      float amp = l / 32768.0f;
      dl->AddLine(ImVec2(footer_min.x + x, mid_y - amp * kWaveFooterH / 2), ImVec2(footer_min.x + x, mid_y + amp * kWaveFooterH / 2), IM_COL32(120, 220, 160, 220));
    }

    double sum_l = 0, sum_r = 0;
    for(int i = 0; i < kN; i++) {
      int16_t l = pcm[(size_t)i * channels];
      int16_t r = channels > 1 ? pcm[(size_t)i * channels + 1] : l;
      sum_l += (double)l * l;
      sum_r += (double)r * r;
    }
    float rms_l = (float)(std::sqrt(sum_l / kN) / 32768.0);
    float rms_r = (float)(std::sqrt(sum_r / kN) / 32768.0);

    // dBスケールのL/Rレベルメーター。目盛り位置(dB)以下は明るいグラデーション、それ以上は暗いままにする(ミキサー風)
    constexpr float kMinDb = -48.0f; // メーター下端に対応するdB
    auto db_to_t           = [&](float db) { return std::clamp((db - kMinDb) / -kMinDb, 0.0f, 1.0f); };
    auto color_for_db      = [&](float db) -> ImU32 {
      if(db > -3.0f) return IM_COL32(230, 70, 70, 255);  // 0dB付近: 赤(クリップ警告)
      if(db > -9.0f) return IM_COL32(230, 210, 70, 255); // 黄
      return IM_COL32(70, 200, 110, 255);                // 緑
    };
    auto draw_meter = [&](float x0, float level) {
      float mw       = meter_w / 2 - 2;
      float level_db = 20.0f * std::log10(std::max(level, 1e-6f));
      float level_t  = db_to_t(level_db);
      for(int py = 0; py < (int)kWaveFooterH; py++) {
        float t   = 1.0f - (float)py / kWaveFooterH; // 0(下端)-1(上端)
        float db  = kMinDb + t * -kMinDb;
        ImU32 col = color_for_db(db);
        if(t > level_t) { // レベル未達部分は暗く沈める(目盛り帯として見える)
          ImVec4 c4 = ImGui::ColorConvertU32ToFloat4(col);
          col       = ImGui::ColorConvertFloat4ToU32(ImVec4(c4.x * 0.25f, c4.y * 0.25f, c4.z * 0.25f, 0.9f));
        }
        dl->AddRectFilled(ImVec2(x0, footer_min.y + py), ImVec2(x0 + mw, footer_min.y + py + 1), col);
      }
      // 主目盛り線(0, -3, -9, -20dB): バー幅いっぱい
      for(float db : {0.0f, -3.0f, -9.0f, -20.0f}) {
        float y = footer_min.y + kWaveFooterH * (1.0f - db_to_t(db));
        dl->AddLine(ImVec2(x0, y), ImVec2(x0 + mw, y), IM_COL32(0, 0, 0, 180));
      }
      // 副目盛り線(6dB刻み): 短めにバー幅の半分だけ引く
      for(float db = -6.0f; db > kMinDb; db -= 6.0f) {
        float y = footer_min.y + kWaveFooterH * (1.0f - db_to_t(db));
        dl->AddLine(ImVec2(x0, y), ImVec2(x0 + mw * 0.5f, y), IM_COL32(0, 0, 0, 130));
      }
    };
    draw_meter(footer_min.x + wave_w, rms_l);
    draw_meter(footer_min.x + wave_w + meter_w / 2, rms_r);
  }

  // フッター2段目(拡大率/フィット/実寸): AEのビューアフッターを参考
  ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + avail.y + kWaveFooterH));
  ImGui::BeginChild("##viewer_footer", ImVec2(0, kCtrlFooterH), false);
  float zoom_pct = zoom * 100.0f;
  ImGui::SetNextItemWidth(80);
  if(ImGui::DragFloat("##zoom_pct", &zoom_pct, 1.0f, 5.0f, 5000.0f, "%.0f%%")) zoom = zoom_pct / 100.0f;
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_EXPAND " フィット")) reset_view();
  ImGui::SameLine();
  if(ImGui::Button("100%") && base_scale > 0.0f) {
    zoom = 1.0f / base_scale;
    pan  = ImVec2(0, 0);
  }
  ImGui::EndChild();

  ImGui::End();
}
} // namespace mu
