#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <functional>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/anim.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/entity_gizmo.hpp>
#include <movutl/gui/graph_editor_window.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/widgets.hpp>

namespace mu {

bool wd_color_edit(const char* name, Vec4b* col) {
  float cf[4]  = {(*col)[0] / 255.0f, (*col)[1] / 255.0f, (*col)[2] / 255.0f, (*col)[3] / 255.0f};
  bool changed = ImGui::ColorEdit4(name, cf);
  if(changed) *col = Vec4b(cf[0] * 255, cf[1] * 255, cf[2] * 255, cf[3] * 255);
  return changed;
}

namespace {
struct EaseOption {
  AniInterpType type;
  const char* name;
};
constexpr EaseOption kEaseOptions[] = {
  {AniInterpType::LINEAR, "Linear"},
  {AniInterpType::EaseInSine, "In Sine"},
  {AniInterpType::EaseOutSine, "Out Sine"},
  {AniInterpType::EaseInOutSine, "InOut Sine"},
  {AniInterpType::EaseInQuad, "In Quad"},
  {AniInterpType::EaseOutQuad, "Out Quad"},
  {AniInterpType::EaseInOutQuad, "InOut Quad"},
  {AniInterpType::EaseInCubic, "In Cubic"},
  {AniInterpType::EaseOutCubic, "Out Cubic"},
  {AniInterpType::EaseInOutCubic, "InOut Cubic"},
  {AniInterpType::EaseInQuart, "In Quart"},
  {AniInterpType::EaseOutQuart, "Out Quart"},
  {AniInterpType::EaseInOutQuart, "InOut Quart"},
  {AniInterpType::EaseInQuint, "In Quint"},
  {AniInterpType::EaseOutQuint, "Out Quint"},
  {AniInterpType::EaseInOutQuint, "InOut Quint"},
  {AniInterpType::EaseInExpo, "In Expo"},
  {AniInterpType::EaseOutExpo, "Out Expo"},
  {AniInterpType::EaseInOutExpo, "InOut Expo"},
  {AniInterpType::EaseInCirc, "In Circ"},
  {AniInterpType::EaseOutCirc, "Out Circ"},
  {AniInterpType::EaseInOutCirc, "InOut Circ"},
  {AniInterpType::EaseInBack, "In Back"},
  {AniInterpType::EaseOutBack, "Out Back"},
  {AniInterpType::EaseInOutBack, "InOut Back"},
  {AniInterpType::EaseInElastic, "In Elastic"},
  {AniInterpType::EaseOutElastic, "Out Elastic"},
  {AniInterpType::EaseInOutElastic, "InOut Elastic"},
  {AniInterpType::EaseInBounce, "In Bounce"},
  {AniInterpType::EaseOutBounce, "Out Bounce"},
  {AniInterpType::EaseInOutBounce, "InOut Bounce"},
  {AniInterpType::Custom, "Custom (Bezier)"},
};
// 旧EaseIn/EaseOut/EaseInOut(EaseInSine等と同式のレガシー別名)はコンボ非表示、見つからない場合はLinear扱いで表示する
const char* ease_name(AniInterpType t) {
  for(auto& o : kEaseOptions)
    if(o.type == t) return o.name;
  return "Linear";
}

// イージング曲線のサムネイル(名前+曲線)。クリックされたらtrue
bool draw_ease_thumb(AniInterpType type, const char* name, bool selected) {
  const ImVec2 size(58, 52);
  ImVec2 o = ImGui::GetCursorScreenPos();
  ImGui::PushID((int)type);
  bool clicked = ImGui::InvisibleButton("##ease_thumb", size);
  ImGui::PopID();
  bool hovered   = ImGui::IsItemHovered();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImVec2 mx(o.x + size.x, o.y + size.y);
  dl->AddRectFilled(o, mx, hovered ? IM_COL32(90, 90, 100, 255) : IM_COL32(60, 60, 66, 255));
  if(selected) dl->AddRect(o, mx, IM_COL32(80, 150, 255, 255), 0.0f, 0, 2.0f);
  dl->PushClipRect(o, mx, true); // 名前がセル幅を超える場合は切り詰める
  dl->AddText(ImVec2(o.x + 3, o.y + 1), IM_COL32(230, 230, 230, 255), name);
  dl->PopClipRect();
  const float x0 = o.x + 7, x1 = mx.x - 7, y0 = mx.y - 7, y1 = o.y + 18; // y0=値0, y1=値1(オーバーシュート用に上下へ余白)
  dl->PushClipRect(ImVec2(o.x, o.y + 11), mx, true);
  ImVec2 prev;
  for(int i = 0; i <= 40; i++) {
    float t = i / 40.0f;
    float v = (float)detail::apply_ease(type, t);
    ImVec2 pt(x0 + (x1 - x0) * t, y0 + (y1 - y0) * v);
    if(i > 0) dl->AddLine(prev, pt, IM_COL32(90, 160, 255, 255), 1.5f);
    prev = pt;
  }
  dl->PopClipRect();
  return clicked;
}
} // namespace

bool wd_bezier_handle_editor(std::array<float, 4>& v, float size) {
  bool changed  = false;
  ImVec2 origin = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("##bezier_bg", ImVec2(size, size));
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(origin, ImVec2(origin.x + size, origin.y + size), IM_COL32(30, 30, 30, 255));
  dl->AddRect(origin, ImVec2(origin.x + size, origin.y + size), IM_COL32(90, 90, 90, 255));

  auto to_screen = [&](float x, float y) { return ImVec2(origin.x + x * size, origin.y + (1.0f - y) * size); };
  ImVec2 p0 = to_screen(0, 0), p3 = to_screen(1, 1);
  ImVec2 p1 = to_screen(v[0], v[1]), p2 = to_screen(v[2], v[3]);
  dl->AddLine(p0, p1, IM_COL32(120, 120, 120, 255));
  dl->AddLine(p3, p2, IM_COL32(120, 120, 120, 255));
  dl->AddBezierCubic(p0, p1, p2, p3, IM_COL32(255, 190, 40, 255), 2.0f);
  dl->AddCircleFilled(p0, 3.0f, IM_COL32(180, 180, 180, 255));
  dl->AddCircleFilled(p3, 3.0f, IM_COL32(180, 180, 180, 255));

  auto handle = [&](const char* id, float* hx, float* hy, ImVec2 screen_pos) {
    ImGui::SetCursorScreenPos(ImVec2(screen_pos.x - 5, screen_pos.y - 5));
    ImGui::PushID(id);
    ImGui::InvisibleButton("##h", ImVec2(10, 10));
    if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      ImVec2 mp = ImGui::GetMousePos();
      *hx       = std::clamp((mp.x - origin.x) / size, 0.0f, 1.0f);
      *hy       = 1.0f - (mp.y - origin.y) / size; // yはオーバーシュート表現のためclampしない
      changed   = true;
    }
    ImGui::PopID();
    dl->AddCircleFilled(screen_pos, 4.0f, IM_COL32(255, 255, 255, 255));
  };
  handle("p1", &v[0], &v[1], p1);
  handle("p2", &v[2], &v[3], p2);
  ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + size + 4));
  return changed;
}

namespace {
// 型ごとに1ウィジェットだけを描画しanim[idx]のframe位置のキーフレームを読み書きするヘルパー(左右スライダーで共用)
template <typename T> bool draw_anim_value_widget(const char* id, const cutil::PropInfo::Field& f, AnimProps& anim, int idx, uint32_t frame) {
  T v = anim.get<T>(idx, frame);
  bool edited;
  if constexpr(std::is_same_v<T, float>) {
    bool has_range = !(f.min_value == 0 && f.max_value == 0);
    if(std::string(f.name) == "alpha_") { // 内部値は0-1だが表示・編集は%
      float pct = v * 100.0f;
      edited    = ImGui::DragFloat(id, &pct, 1.0f, 0.0f, 100.0f, "%.0f%%");
      if(edited) v = std::clamp(pct, 0.0f, 100.0f) / 100.0f;
    } else {
      edited = has_range ? ImGui::SliderFloat(id, &v, f.min_value, f.max_value) : ImGui::DragFloat(id, &v, f.drag_speed);
    }
  } else if constexpr(std::is_same_v<T, int>) {
    bool has_range = !(f.min_value == 0 && f.max_value == 0);
    edited         = has_range ? ImGui::SliderInt(id, &v, (int)f.min_value, (int)f.max_value) : ImGui::DragInt(id, &v, f.drag_speed);
  } else if constexpr(std::is_same_v<T, bool>) {
    edited = ImGui::Checkbox(id, &v);
  } else if constexpr(std::is_same_v<T, Vec2>) {
    edited = ImGui::DragFloat2(id, v.value, f.drag_speed);
  } else if constexpr(std::is_same_v<T, Vec3>) {
    edited = ImGui::DragFloat3(id, v.value, f.drag_speed);
  } else if constexpr(std::is_same_v<T, Vec4>) {
    edited = ImGui::DragFloat4(id, v.value, f.drag_speed);
  } else if constexpr(std::is_same_v<T, Vec4b>) {
    edited = wd_color_edit(id, &v);
  } else {
    edited = false;
  }
  if(edited) anim.add_keyframe<T>(idx, frame, v, anim.get_ease_type(idx, frame));
  return edited;
}

// 実行時のcutil::PropInfo*からdraw_anim_value_widget<T>を選んで呼ぶディスパッチャ
bool draw_anim_value_widget_dyn(const char* id, const cutil::PropInfo::Field& f, AnimProps& anim, int idx, uint32_t frame) {
  const cutil::PropInfo* type = anim.get_type(idx);
  if(type == cutil::prop_info_of<float>()) return draw_anim_value_widget<float>(id, f, anim, idx, frame);
  if(type == cutil::prop_info_of<int>()) return draw_anim_value_widget<int>(id, f, anim, idx, frame);
  if(type == cutil::prop_info_of<bool>()) return draw_anim_value_widget<bool>(id, f, anim, idx, frame);
  if(type == cutil::prop_info_of<Vec2>()) return draw_anim_value_widget<Vec2>(id, f, anim, idx, frame);
  if(type == cutil::prop_info_of<Vec3>()) return draw_anim_value_widget<Vec3>(id, f, anim, idx, frame);
  if(type == cutil::prop_info_of<Vec4>()) return draw_anim_value_widget<Vec4>(id, f, anim, idx, frame);
  if(type == cutil::prop_info_of<Vec4b>()) return draw_anim_value_widget<Vec4b>(id, f, anim, idx, frame);
  return false;
}
} // namespace

bool wd_animatable_row(const cutil::PropInfo::Field& f, AnimProps& anim, int idx, uint32_t cur_frame, uint64_t entity_guid, int filter_index, int length) {
  ImGui::PushID(f.name);
  bool changed        = false;
  auto [pf, nf]       = anim.neighbor_frames(idx, cur_frame);
  const char* label   = f.label[0] ? f.label : f.name;
  const bool animated = anim.has_animation(idx);

  float avail    = ImGui::GetContentRegionAvail().x;
  float spacing  = ImGui::GetStyle().ItemSpacing.x;
  float side_w   = avail * 0.26f;
  float center_w = avail - side_w * 2 - spacing * 2;

  ImGui::SetNextItemWidth(side_w);
  if(draw_anim_value_widget_dyn("##l", f, anim, idx, pf)) changed = true;
  ImGui::SameLine();

  if(animated) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.65f, 0.15f, 1.0f));
  bool clicked = ImGui::Button(label, ImVec2(center_w, 0));
  if(animated) ImGui::PopStyleColor();
  if(clicked) ImGui::OpenPopup("##anim_popup");

  if(ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    GraphDragPayload payload;
    payload.entity_guid  = entity_guid;
    payload.filter_index = filter_index;
    strncpy(payload.prop_name, f.name, sizeof(payload.prop_name) - 1);
    ImGui::SetDragDropPayload(kGraphDragDropId, &payload, sizeof(payload));
    ImGui::Text("%s", label);
    ImGui::EndDragDropSource();
  }

  bool open_settings = false; // OpenPopupはポップアップ内側だとIDスコープがずれるため、EndPopup後に開く
  if(ImGui::BeginPopup("##anim_popup")) {
    // AviUtl準拠: 移動方式メニュー。未アニメ時に選ぶとfstart〜fendの2点キーで区間アニメを開始する
    const AniInterpType cur = animated ? anim.get_ease_type(idx, pf) : AniInterpType::LINEAR;
    auto pick               = [&](AniInterpType t) {
      if(!animated) {
        uint32_t a = 0;
        uint32_t b = length > 0 ? (uint32_t)length : 30;
        anim.add_keyframe_here(idx, a);
        anim.add_keyframe_here(idx, b);
        pf = a;
      }
      anim.set_ease_type(idx, pf, t);
      changed = true;
    };
    if(ImGui::MenuItem("移動無し", nullptr, !animated) && animated) {
      anim.collapse_to_single(idx, cur_frame);
      changed = true;
    }
    struct Preset {
      const char* name;
      AniInterpType type;
    };
    static constexpr Preset kPresets[] = {{"直線移動", AniInterpType::LINEAR}, {"加減速移動", AniInterpType::EaseInOutSine}, {"曲線移動", AniInterpType::Custom}, {"加速", AniInterpType::EaseInQuad}, {"減速", AniInterpType::EaseOutQuad}};
    for(auto& pr : kPresets)
      if(ImGui::MenuItem(pr.name, nullptr, animated && cur == pr.type)) pick(pr.type);
    if(ImGui::BeginMenu("easing")) {
      int n = 0;
      for(auto& opt : kEaseOptions) {
        if(opt.type == AniInterpType::Custom) continue; // ベジエは「曲線移動」から
        if(n++ % 5 != 0) ImGui::SameLine();
        if(draw_ease_thumb(opt.type, opt.name, animated && cur == opt.type)) {
          pick(opt.type);
          ImGui::CloseCurrentPopup();
        }
      }
      ImGui::EndMenu();
    }
    ImGui::Separator();
    if(animated) {
      if(anim.has_key_at(idx, cur_frame)) {
        if(ImGui::MenuItem(ICON_FA_TRASH " このフレームの中間点を削除")) {
          anim.erase_keyframe(idx, cur_frame);
          changed = true;
        }
      } else if(ImGui::MenuItem(ICON_FA_DIAMOND " このフレームに中間点を追加")) {
        anim.add_keyframe_here(idx, cur_frame);
        changed = true;
      }
      if(cur == AniInterpType::Custom && ImGui::MenuItem("設定")) open_settings = true;
    }
    ImGui::EndPopup();
  }
  if(open_settings) ImGui::OpenPopup("##anim_settings");
  if(ImGui::BeginPopup("##anim_settings")) {
    auto bez = anim.get_ease_bezier(idx, pf);
    if(wd_bezier_handle_editor(bez, 120.0f)) {
      anim.set_ease_bezier(idx, pf, bez);
      changed = true;
    }
    ImGui::EndPopup();
  }
  ImGui::SameLine();

  ImGui::SetNextItemWidth(side_w);
  if(draw_anim_value_widget_dyn("##r", f, anim, idx, nf)) changed = true;

  ImGui::PopID();
  return changed;
}

bool wd_entity_keyframe_overview(Entity* e, uint32_t cur_frame) {
  MU_ASSERT(e);
  ImGui::PushID("##kf_overview");
  bool seeked     = false;
  auto frames     = e->collect_animated_frames();
  auto jump_frame = [&](bool forward) -> int {
    int cur    = (int)cur_frame;
    int best   = forward ? e->fend_ : e->fstart_;
    bool found = false;
    for(uint32_t fr : frames) {
      if(forward && (int)fr > cur && (!found || (int)fr < best)) {
        best  = (int)fr;
        found = true;
      }
      if(!forward && (int)fr < cur && (!found || (int)fr > best)) {
        best  = (int)fr;
        found = true;
      }
    }
    return best;
  };

  ImGui::Text("%d", e->fstart_);
  ImGui::SameLine();
  if(ImGui::SmallButton(ICON_FA_BACKWARD_STEP)) {
    if(auto* comp = e->get_comp()) {
      comp->set_frame(jump_frame(false));
      seeked = true;
    }
  }
  ImGui::SameLine();

  const float height = 18.0f;
  ImVec2 origin      = ImGui::GetCursorScreenPos();
  float width        = ImGui::GetContentRegionAvail().x - 60.0f; // 右端のジャンプボタン+終了フレーム表示分を空けておく
  width              = std::max(width, 20.0f);
  ImGui::InvisibleButton("##overview_bg", ImVec2(width, height));
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height), IM_COL32(40, 40, 40, 180));

  const int range = std::max(e->fend_ - e->fstart_, 1);
  auto frame_to_x = [&](int frame) { return origin.x + width * std::clamp((float)(frame - e->fstart_) / (float)range, 0.0f, 1.0f); };

  if(ImGui::IsItemActive() || (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left))) {
    float mx      = ImGui::GetMousePos().x;
    int new_frame = e->fstart_ + (int)std::round((mx - origin.x) / width * range);
    new_frame     = std::clamp(new_frame, e->fstart_, e->fend_);
    if(auto* comp = e->get_comp()) {
      comp->set_frame(new_frame);
      seeked = true;
    }
  }

  for(uint32_t kf : frames) {
    float x = frame_to_x((int)kf);
    ImVec2 center(x, origin.y + height * 0.5f);
    dl->AddQuadFilled(ImVec2(center.x, center.y - 5), ImVec2(center.x + 5, center.y), ImVec2(center.x, center.y + 5), ImVec2(center.x - 5, center.y), IM_COL32(255, 170, 40, 255));
  }
  float cx = frame_to_x((int)cur_frame);
  dl->AddLine(ImVec2(cx, origin.y), ImVec2(cx, origin.y + height), IM_COL32(255, 255, 255, 200), 2.0f);

  ImGui::SameLine();
  if(ImGui::SmallButton(ICON_FA_FORWARD_STEP)) {
    if(auto* comp = e->get_comp()) {
      comp->set_frame(jump_frame(true));
      seeked = true;
    }
  }
  ImGui::SameLine();
  ImGui::Text("%d", e->fend_);

  ImGui::PopID();
  return seeked;
}

bool wd_table_begin(const char* id) {
  if(!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_PadOuterX)) return false;
  ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthStretch, 0.4f);
  ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthStretch, 0.6f);
  return true;
}

void wd_table_end() { ImGui::EndTable(); }

void wd_row(const char* label, const char* desc) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  // 幅が足りないラベルは省略記号で切る(隣の値欄へはみ出さない)。全文はツールチップで見せる
  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const float w    = std::max(1.0f, ImGui::GetContentRegionAvail().x);
  const float h    = ImGui::GetTextLineHeight();
  ImGui::RenderTextEllipsis(ImGui::GetWindowDrawList(), pos, ImVec2(pos.x + w, pos.y + h), pos.x + w, pos.x + w, label, nullptr, nullptr);
  ImGui::Dummy(ImVec2(w, h));
  if(ImGui::IsItemHovered() && (desc && desc[0] || ImGui::CalcTextSize(label).x > w)) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted(label);
    if(desc && desc[0]) ImGui::TextDisabled("%s", desc);
    ImGui::EndTooltip();
  }
  ImGui::TableSetColumnIndex(1);
  ImGui::SetNextItemWidth(-FLT_MIN);
}

bool wd_grid9(const char* id, int selected, int* picked, const char* const* tips) {
  bool clicked           = false;
  const float avail      = ImGui::GetContentRegionAvail().x;
  const float sp         = 2.0f;
  const float cell       = std::clamp((avail - sp * 2) / 3.0f, 16.0f, 30.0f);
  const ImVec2 origin    = ImGui::GetCursorScreenPos();
  auto* dl               = ImGui::GetWindowDrawList();
  const ImU32 col_sel    = ImGui::GetColorU32(ImGuiCol_ButtonActive);
  const ImU32 col_hov    = ImGui::GetColorU32(ImGuiCol_ButtonHovered);
  const ImU32 col_bg     = ImGui::GetColorU32(ImGuiCol_FrameBg);
  const ImU32 col_border = ImGui::GetColorU32(ImGuiCol_Border);
  ImGui::PushID(id);
  for(int i = 0; i < 9; i++) {
    const int cx = i % 3, cy = i / 3;
    const ImVec2 mn(origin.x + cx * (cell + sp), origin.y + cy * (cell * 0.75f + sp));
    const ImVec2 mx(mn.x + cell, mn.y + cell * 0.75f);
    ImGui::SetCursorScreenPos(mn);
    ImGui::PushID(i);
    ImGui::InvisibleButton("##c", ImVec2(cell, cell * 0.75f));
    const bool hov = ImGui::IsItemHovered();
    if(ImGui::IsItemClicked()) {
      if(picked) *picked = i;
      clicked = true;
    }
    if(hov && tips && tips[i]) ImGui::SetTooltip("%s", tips[i]);
    ImGui::PopID();
    dl->AddRectFilled(mn, mx, i == selected ? col_sel : (hov ? col_hov : col_bg), 3.0f);
    dl->AddRect(mn, mx, col_border, 3.0f);
    // 枠内の該当位置に点を打つ(左上/上/右上/左/中央/右/左下/下/右下)
    const float pad = 4.0f, d = 3.0f;
    const float px = cx == 0 ? mn.x + pad : cx == 1 ? (mn.x + mx.x) * 0.5f - d * 0.5f : mx.x - pad - d;
    const float py = cy == 0 ? mn.y + pad : cy == 1 ? (mn.y + mx.y) * 0.5f - d * 0.5f : mx.y - pad - d;
    dl->AddRectFilled(ImVec2(px, py), ImVec2(px + d, py + d), i == selected ? IM_COL32(255, 255, 255, 255) : IM_COL32(200, 200, 200, 180));
  }
  ImGui::PopID();
  ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + 3 * (cell * 0.75f + sp)));
  return clicked;
}

namespace {
// infoの各フィールドを編集UIとして描画し、変更があればapply(変更分のProp)で反映する。
// bool/int/float/Vec2/Vec3/Vec4/Vec4bはanim_props_(中間点)経由でフル幅のトラックバー行(左右=直前/直後の中間点値、中央=名前ボタン)、
// それ以外(string/path/uint8_t/列挙)は「ラベル左・値右」の表でapply(setProps/setTransformProps)経由で編集する
void edit_props(Entity* e, const cutil::PropInfo* info, const cutil::Prop& p, uint32_t cur_frame, const std::function<void(const cutil::Prop&)>& apply) {
  if(!info) return;
  e->ensure_anim_props();
  const uint32_t rel_frame = e->rel_frame((int)cur_frame); // anim_props_の中間点はトラック開始からの相対frame
  bool in_table            = false;
  int table_seq            = 0;
  auto ensure_table        = [&](bool want) {
    if(want == in_table) return;
    if(want) {
      in_table = wd_table_begin(("##props" + std::to_string(table_seq++)).c_str());
    } else {
      wd_table_end();
      in_table = false;
    }
  };
  for(const auto& f : info->fields) {
    if(!p.contains(f.name)) {
      LOG_F(WARNING, "Property %s -> %s not found", e->name.c_str(), f.name);
      continue;
    }
    bool changed = false;
    cutil::Prop newp;
    const char* label_       = f.label[0] ? f.label : f.name;
    const char* name_        = "##v"; // ラベルは左の列(wd_row)に出すので、ウィジェット側のラベルは隠す
    const bool is_path_field = std::string(f.name) == "path" || std::string(f.name) == "path_";
    const int anim_idx       = e->anim_props_.index_of(f.name);
    const bool is_animatable = anim_idx >= 0;
    const bool is_enum       = std::string(f.name) == "shape_type_"; // Comboで選ぶ列挙なのでトラックバーUIの対象外
    const bool use_trackbar =
      is_animatable && !is_enum &&
      (f.type == cutil::prop_info_of<bool>() || f.type == cutil::prop_info_of<float>() || f.type == cutil::prop_info_of<int32_t>() || f.type == cutil::prop_info_of<Vec2>() || f.type == cutil::prop_info_of<Vec3>() || f.type == cutil::prop_info_of<Vec4>() || f.type == cutil::prop_info_of<Vec4b>());

    ensure_table(!use_trackbar); // 表の開閉はPushIDの外で行う(ID stackが食い違うとEndTableでassertする)
    ImGui::PushID(f.name);
    if(use_trackbar) {
      if(wd_animatable_row(f, e->anim_props_, anim_idx, rel_frame, e->guid_, -1, e->fend_ - e->fstart_)) changed = true;
    } else {
      if(in_table) wd_row(label_, f.desc);
      if(f.type == cutil::prop_info_of<int32_t>() && is_enum) {
        static const char* kShapeNames[] = {"三角形", "四角形", "六角形", "円", "カスタムパス"};
        int idx                          = std::clamp(p.get<int32_t>(f.name), 0, 4);
        if(ImGui::Combo(name_, &idx, kShapeNames, IM_ARRAYSIZE(kShapeNames))) {
          if(is_animatable)
            e->anim_props_.set_value<int>(anim_idx, rel_frame, idx);
          else
            newp.set<int32_t>(f.name, idx);
          changed = true;
        }
      } else if(f.type == cutil::prop_info_of<uint8_t>()) {
        int v = p.get<uint8_t>(f.name);
        if(ImGui::InputInt(name_, &v)) {
          newp.set<uint8_t>(f.name, static_cast<uint8_t>(std::clamp(v, 0, 255)));
          changed = true;
        }
      } else if(f.type == cutil::prop_info_of<std::string>()) {
        std::string s = p.get<std::string>(f.name);
        char buf[256];
        strncpy(buf, s.c_str(), sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        if(is_path_field) {
          std::string label = s.empty() ? "ファイルを選択" : std::filesystem::path(s).filename().string();
          std::string btn   = std::string(ICON_FA_FOLDER_OPEN " ") + label;
          if(ImGui::Button(btn.c_str(), ImVec2(-FLT_MIN, 0))) {
            std::string picked = select_file_dialog("ファイルを選択", {});
            if(!picked.empty()) {
              newp.set<std::string>(f.name, picked);
              changed = true;
            }
          }
        } else if(ImGui::InputText(name_, buf, sizeof(buf))) {
          newp.set<std::string>(f.name, std::string(buf));
          changed = true;
        }
      }
    }

    if(changed) {
      {
        std::lock_guard<std::mutex> lock(e->mtx);
        if(is_animatable && (use_trackbar || is_enum))
          e->apply_animated_props((int)cur_frame); // anim_props_の編集結果をメンバ変数へ反映する
        else
          apply(newp);
        if(is_path_field) {
          e->reload_asset(); // パス変更時は新しいファイルを読み込み直す
          auto new_path = newp.get<std::string>(f.name);
          if(!new_path.empty()) e->name = std::filesystem::path(new_path).stem().string();
        }
      }
      // Compositionの全フレームではなく、このEntityが映る範囲だけを無効化する(Positionドラッグ等が重くなるのを防ぐ)
      if(auto* comp = e->get_comp()) comp->invalidate_cache_range(e->fstart_, e->fend_);
    }
    ImGui::PopID();
  }
  ensure_table(false);
}

// 基点を画像枠上の9点(左上〜右下)へ置くプリセット。keep_visualなら見た目が動かないようposも補正する。現在の基点がいずれかの点と一致すれば選択表示にする
void edit_anchor_presets(Entity* e) {
  static bool keep_visual = true;
  auto* comp              = e->get_comp();
  if(!comp) return;
  static const char* kTips[9] = {"左上", "上", "右上", "左", "中央", "右", "左下", "下", "右下"};
  EntityGizmo g;
  if(!entity_gizmo_of(*e, GizmoPt{(double)comp->size[0], (double)comp->size[1]}, g)) return;
  int selected = -1;
  for(int i = 0; i < 9; i++) {
    const GizmoPt a = gizmo_anchor_preset(i % 3 - 1, i / 3 - 1, g.src_size, g.origin_offset);
    if(std::abs(a.x - g.xform.anchor.x) < 0.5 && std::abs(a.y - g.xform.anchor.y) < 0.5) selected = i;
  }
  if(!wd_table_begin("##anchor_presets")) return;
  wd_row("基点プリセット");
  int picked = -1;
  if(wd_grid9("##anchor_grid", selected, &picked, kTips)) {
    const GizmoPt anchor = gizmo_anchor_preset(picked % 3 - 1, picked / 3 - 1, g.src_size, g.origin_offset);
    {
      std::lock_guard<std::mutex> lock(e->mtx);
      entity_apply_xform(*e, gizmo_set_anchor(g.xform, anchor, keep_visual));
    }
    comp->invalidate_cache_range(e->fstart_, e->fend_);
  }
  wd_row("");
  ImGui::Checkbox("位置を保持", &keep_visual);
  if(ImGui::IsItemHovered()) ImGui::SetTooltip("基点を動かしても見た目の位置が変わらないよう位置を補正する");
  wd_table_end();
}
} // namespace

void wd_entt_transform_editor(Entity* e, uint32_t cur_frame) {
  MU_ASSERT(e);
  if(!e->has_transform()) return;
  ImGui::PushID(e);
  edit_props(e, e->getTransformPropsInfo(), e->getTransformProps(), cur_frame, [&](const cutil::Prop& np) { e->setTransformProps(np); });
  edit_anchor_presets(e);
  ImGui::PopID();
}

void wd_entt_props_editor(Entity* e, uint32_t cur_frame) {
  MU_ASSERT(e);
  ImGui::PushID(e);
  edit_props(e, e->getPropsInfo(), e->getProps(), cur_frame, [&](const cutil::Prop& np) { e->setProps(np); });
  ImGui::PopID();
}

void wd_movie_inspector(Entity* e) { MU_ASSERT(e); }

void wd_image_inspector(Entity* e) { MU_ASSERT(e); }

} // namespace mu
