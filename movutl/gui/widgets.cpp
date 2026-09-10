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

bool wd_keyframe_toggle(AnimProps& anim, int idx, uint32_t cur_frame) {
  bool has_key  = anim.has_key_at(idx, cur_frame);
  bool animated = anim.has_animation(idx);
  ImVec4 col    = has_key ? ImVec4(1.0f, 0.65f, 0.15f, 1.0f) : (animated ? ImVec4(1.0f, 0.65f, 0.15f, 0.35f) : ImVec4(0.6f, 0.6f, 0.6f, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_Text, col);
  bool clicked = ImGui::SmallButton(has_key ? ICON_FA_DIAMOND : ICON_FA_CIRCLE);
  ImGui::PopStyleColor();
  if(ImGui::IsItemHovered()) ImGui::SetTooltip(has_key ? "中間点を削除" : "中間点を追加");
  if(!clicked) return false;
  if(has_key) return anim.erase_keyframe(idx, cur_frame);
  anim.add_keyframe_here(idx, cur_frame);
  return true;
}

bool wd_keyframe_strip(const char* str_id, AnimProps& anim, int idx, int fstart, int fend, uint32_t cur_frame) {
  ImGui::PushID(str_id);
  bool changed       = false;
  const float height = 14.0f;
  ImVec2 origin      = ImGui::GetCursorScreenPos();
  float width        = ImGui::GetContentRegionAvail().x;
  ImGui::InvisibleButton("##strip_bg", ImVec2(width, height));
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height), IM_COL32(40, 40, 40, 180));

  const int range = std::max(fend - fstart, 1);
  auto frame_to_x = [&](int frame) { return origin.x + width * std::clamp((float)(frame - fstart) / (float)range, 0.0f, 1.0f); };

  // 現在フレーム位置
  float cx = frame_to_x((int)cur_frame);
  dl->AddLine(ImVec2(cx, origin.y), ImVec2(cx, origin.y + height), IM_COL32(255, 255, 255, 150));

  auto frames = anim.keyframe_frames(idx);
  for(int ki = 0; ki < (int)frames.size(); ki++) {
    uint32_t kf = frames[ki];
    float x     = frame_to_x((int)kf);
    ImVec2 center(x, origin.y + height * 0.5f);
    ImU32 col = IM_COL32(255, 170, 40, 255);
    dl->AddQuadFilled(ImVec2(center.x, center.y - 5), ImVec2(center.x + 5, center.y), ImVec2(center.x, center.y + 5), ImVec2(center.x - 5, center.y), col);

    ImGui::SetCursorScreenPos(ImVec2(center.x - 5, center.y - 5));
    ImGui::PushID(ki); // frame値(kf)はドラッグ中に変化しIDが不安定になるため、配列indexを使う(ki自体もソート順の入れ替わりで跨ぐケースはあるが稀)
    ImGui::InvisibleButton("##kf", ImVec2(10, 10));
    if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      float mx      = ImGui::GetMousePos().x;
      int new_frame = fstart + (int)std::round((mx - origin.x) / width * range);
      new_frame     = std::clamp(new_frame, fstart, fend);
      if((uint32_t)new_frame != kf && anim.move_keyframe(idx, kf, (uint32_t)new_frame)) changed = true;
    }
    if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
      if(anim.erase_keyframe(idx, kf)) changed = true;
    }
    if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) ImGui::OpenPopup("##ease_popup");
    if(ImGui::BeginPopup("##ease_popup")) {
      AniInterpType cur = anim.get_ease_type(idx, kf);
      if(ImGui::BeginCombo("イージング", ease_name(cur))) {
        for(auto& opt : kEaseOptions) {
          bool selected = opt.type == cur;
          if(ImGui::Selectable(opt.name, selected)) {
            anim.set_ease_type(idx, kf, opt.type);
            changed = true;
          }
        }
        ImGui::EndCombo();
      }
      if(cur == AniInterpType::Custom) {
        auto bez = anim.get_ease_bezier(idx, kf);
        if(wd_bezier_handle_editor(bez, 120.0f)) {
          anim.set_ease_bezier(idx, kf, bez);
          changed = true;
        }
      }
      ImGui::EndPopup();
    }
    ImGui::PopID();
  }
  ImGui::PopID();
  return changed;
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

    if(f.type == cutil::prop_info_of<bool>()) {
      bool v = p.get<bool>(f.name);
      if(ImGui::Checkbox(name_, &v)) {
        e->anim_props_.set_value<bool>(anim_idx, cur_frame, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<float>()) {
      float v = p.get<float>(f.name);
      if(ImGui::DragFloat(name_, &v, f.drag_speed, f.min_value, f.max_value)) {
        e->anim_props_.set_value<float>(anim_idx, cur_frame, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<int32_t>()) {
      int32_t v = p.get<int32_t>(f.name);
      if(std::string(f.name) == "shape_type_") {
        static const char* kShapeNames[] = {"三角形", "四角形", "六角形", "円", "カスタムパス"};
        int shape_idx                    = std::clamp(v, 0, 4);
        if(ImGui::Combo(name_, &shape_idx, kShapeNames, IM_ARRAYSIZE(kShapeNames))) {
          e->anim_props_.set_value<int>(anim_idx, cur_frame, shape_idx);
          changed = true;
        }
      } else if(ImGui::InputInt(name_, &v)) {
        e->anim_props_.set_value<int>(anim_idx, cur_frame, v);
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
    } else if(f.type == cutil::prop_info_of<Vec2>()) {
      Vec2 v = p.get<Vec2>(f.name);
      if(ImGui::DragFloat2(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        e->anim_props_.set_value<Vec2>(anim_idx, cur_frame, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec3>()) {
      Vec3 v = p.get<Vec3>(f.name);
      if(ImGui::DragFloat3(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        e->anim_props_.set_value<Vec3>(anim_idx, cur_frame, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec4>()) {
      Vec4 v = p.get<Vec4>(f.name);
      if(ImGui::DragFloat4(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        e->anim_props_.set_value<Vec4>(anim_idx, cur_frame, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec4b>()) {
      Vec4b v = p.get<Vec4b>(f.name);
      if(wd_color_edit(name_, &v)) {
        e->anim_props_.set_value<Vec4b>(anim_idx, cur_frame, v);
        changed = true;
      }
    }

    if(is_animatable && ImGui::BeginDragDropSource()) {
      GraphDragPayload payload;
      payload.entity_guid  = e->guid_;
      payload.filter_index = -1;
      strncpy(payload.prop_name, f.name, sizeof(payload.prop_name) - 1);
      ImGui::SetDragDropPayload(kGraphDragDropId, &payload, sizeof(payload));
      ImGui::Text("%s", name_);
      ImGui::EndDragDropSource();
    }

    if(is_animatable) {
      ImGui::SameLine();
      if(wd_keyframe_toggle(e->anim_props_, anim_idx, cur_frame)) changed = true;
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

    if(is_animatable && e->anim_props_.has_animation(anim_idx)) {
      if(wd_keyframe_strip(f.name, e->anim_props_, anim_idx, e->fstart_, e->fend_, cur_frame)) {
        if(auto* comp = e->get_comp()) comp->invalidate_cache_range(e->fstart_, e->fend_);
      }
    }
    ImGui::PopID();
  }
  ImGui::PopID();
}

void wd_movie_inspector(Entity* e) { MU_ASSERT(e); }

void wd_image_inspector(Entity* e) { MU_ASSERT(e); }

} // namespace mu
