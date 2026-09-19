#include <IconsFontAwesome6.h>
#include <algorithm>
#include <filesystem>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/anim.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
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
    edited         = has_range ? ImGui::SliderFloat(id, &v, f.min_value, f.max_value) : ImGui::DragFloat(id, &v, f.drag_speed);
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

bool wd_animatable_row(const cutil::PropInfo::Field& f, AnimProps& anim, int idx, uint32_t cur_frame, uint64_t entity_guid, int filter_index, int fstart, int fend) {
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
        uint32_t a = fstart >= 0 ? (uint32_t)fstart : cur_frame;
        uint32_t b = fend > fstart && fend >= 0 ? (uint32_t)fend : a + 30;
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
      for(auto& opt : kEaseOptions)
        if(ImGui::MenuItem(opt.name, nullptr, animated && cur == opt.type)) pick(opt.type);
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

// bool/int/float/Vec2/Vec3/Vec4/Vec4bはanim_props_(中間点)経由、それ以外(string/path/uint8_t)は従来通りgetProps/setProps経由で編集する
void wd_entt_props_editor(Entity* e, uint32_t cur_frame) {
  MU_ASSERT(e);
  ImGui::PushID(e);

  const cutil::PropInfo* info = e->getPropsInfo();
  if(!info) {
    ImGui::PopID();
    return;
  }

  e->ensure_anim_props();
  const auto p = e->getProps();
  for(int idx = 0; idx < (int)info->fields.size(); idx++) {
    const auto& f = info->fields[idx];
    if(!p.contains(f.name)) {
      LOG_F(WARNING, "Property %s -> %s not found", e->name.c_str(), f.name);
      continue;
    }
    ImGui::PushID(f.name);
    bool changed = false;
    cutil::Prop newp;
    const char* name_        = f.label[0] ? f.label : f.name;
    const bool is_path_field = std::string(f.name) == "path" || std::string(f.name) == "path_";
    const int anim_idx       = e->anim_props_.index_of(f.name);
    const bool is_animatable = anim_idx >= 0;

    if(is_animatable && f.type == cutil::prop_info_of<int32_t>() && std::string(f.name) == "shape_type_") {
      // shape_type_はComboで選ぶ列挙なのでトラックバーUIの対象外(キーフレームUIなし、既存の直接編集のまま)
      int32_t v                        = p.get<int32_t>(f.name);
      static const char* kShapeNames[] = {"三角形", "四角形", "六角形", "円", "カスタムパス"};
      int shape_idx                    = std::clamp(v, 0, 4);
      if(ImGui::Combo(name_, &shape_idx, kShapeNames, IM_ARRAYSIZE(kShapeNames))) {
        e->anim_props_.set_value<int>(anim_idx, cur_frame, shape_idx);
        changed = true;
      }
    } else if(is_animatable && (f.type == cutil::prop_info_of<bool>() || f.type == cutil::prop_info_of<float>() || f.type == cutil::prop_info_of<int32_t>() || f.type == cutil::prop_info_of<Vec2>() || f.type == cutil::prop_info_of<Vec3>() || f.type == cutil::prop_info_of<Vec4>() ||
                                f.type == cutil::prop_info_of<Vec4b>())) {
      if(wd_animatable_row(f, e->anim_props_, anim_idx, cur_frame, e->guid_, -1, e->fstart_, e->fend_)) changed = true;
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
        if(ImGui::Button(btn.c_str(), ImVec2(-1, 0))) {
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

    if(changed) {
      {
        std::lock_guard<std::mutex> lock(e->mtx);
        if(is_animatable) {
          e->apply_animated_props((int)cur_frame); // anim_props_の編集結果をメンバ変数へ反映する
        } else {
          e->setProps(newp);
        }
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
  ImGui::PopID();
}

void wd_movie_inspector(Entity* e) { MU_ASSERT(e); }

void wd_image_inspector(Entity* e) { MU_ASSERT(e); }

} // namespace mu
