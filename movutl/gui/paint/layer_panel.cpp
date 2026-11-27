#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {
namespace {

// 並びはクリスタの合成モードに合わせる
constexpr std::pair<BlendType, const char*> kBlends[] = {
  {Blend_Alpha, "通常"}, {Blend_Darken, "比較(暗)"}, {Blend_Mul, "乗算"}, {Blend_Sub, "減算"}, {Blend_Lighten, "比較(明)"}, {Blend_Screen, "スクリーン"}, {Blend_Add, "加算"}, {Blend_Overlay, "オーバーレイ"}, {Blend_HardLight, "ハードライト"}, {Blend_Div, "除算"},
};

const char* blend_name(BlendType b) {
  for(const auto& [type, name] : kBlends)
    if(type == b) return name;
  return "通常";
}

int percent(float v) { return (int)std::lround(v * 100.0f); }

} // namespace

void LayerPanel::Update() {
  if(!ImGui::Begin(paint_window_title("layers").c_str(), &open)) {
    ImGui::End();
    return;
  }
  auto& st           = paint_editor();
  PaintDocument* doc = st.doc();
  if(!doc || doc->layers.empty()) {
    ImGui::TextDisabled("キャンバスがありません");
    ImGui::End();
    return;
  }
  // 覚えている対象が今の文書に残っているか
  const auto alive  = [&](const Target& t) { return t.doc.lock() == doc->handle().lock() && t.index < (int)doc->layers.size(); };
  const int active  = st.active_layer();
  const float width = ImGui::GetContentRegionAvail().x;

  // 合成モードと不透明度
  {
    PaintLayer& L = doc->layers[active];
    ImGui::SetNextItemWidth(width * 0.45f);
    if(ImGui::BeginCombo("##blend", blend_name(L.blend), ImGuiComboFlags_HeightLargest)) {
      for(const auto& [type, name] : kBlends) {
        if(!ImGui::Selectable(name, L.blend == type)) continue;
        LayerProps p = L;
        p.blend      = type;
        st.set_layer_props(active, p);
      }
      ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("合成モード");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    // ドラッグ中は直接書き換えて表示に反映し、離した時に1回分のundoにする
    int opacity = percent(L.opacity);
    if(ImGui::SliderInt("##opacity", &opacity, 0, 100, "不透明度 %d", ImGuiSliderFlags_AlwaysClamp)) {
      if(!drag_ || !alive(*drag_) || drag_->index != active) drag_ = Drag{{doc->handle(), active}, L};
      L.opacity = opacity / 100.0f;
      doc->mark_changed();
    }
    if(drag_ && !ImGui::IsItemActive()) {
      if(alive(*drag_)) {
        LayerProps& cur        = doc->layers[drag_->index];
        const LayerProps after = cur;
        cur                    = drag_->before;
        doc->mark_changed();
        st.set_layer_props(drag_->index, after);
      }
      drag_.reset();
    }
  }

  const int count = (int)doc->layers.size();
  if(ImGui::Button(ICON_FA_FILE_CIRCLE_PLUS "##add")) st.add_layer();
  ImGui::SetItemTooltip("新規ラスターレイヤー");
  ImGui::SameLine();
  ImGui::BeginDisabled(count <= 1);
  if(ImGui::Button(ICON_FA_TRASH_CAN "##remove")) st.remove_layer();
  ImGui::SetItemTooltip("レイヤーを削除");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(st.active_layer() >= (int)doc->layers.size() - 1);
  if(ImGui::Button(ICON_FA_ARROW_UP "##up")) st.move_layer(st.active_layer() + 1);
  ImGui::SetItemTooltip("レイヤーを上へ");
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(st.active_layer() <= 0);
  if(ImGui::Button(ICON_FA_ARROW_DOWN "##down")) st.move_layer(st.active_layer() - 1);
  ImGui::SetItemTooltip("レイヤーを下へ");
  ImGui::EndDisabled();

  int move_from = -1, move_to = -1; // 並びは一覧を描き終えてから変える
  if(ImGui::BeginChild("##list", ImVec2(0, 0), ImGuiChildFlags_Borders)) {
    for(int i = (int)doc->layers.size() - 1; i >= 0; i--) { // 上のレイヤーから
      ImGui::PushID(i);
      const LayerProps props = doc->layers[i];
      if(ImGui::Button(props.visible ? ICON_FA_EYE "##visible" : "##visible", ImVec2(ImGui::GetFrameHeight() * 1.4f, 0))) {
        LayerProps p = props;
        p.visible    = !p.visible;
        st.set_layer_props(i, p);
      }
      ImGui::SetItemTooltip(props.visible ? "レイヤーを非表示" : "レイヤーを表示");
      ImGui::SameLine();
      if(rename_ && alive(*rename_) && rename_->index == i) {
        if(focus_rename_) ImGui::SetKeyboardFocusHere();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputText("##name", name_, sizeof(name_), ImGuiInputTextFlags_AutoSelectAll);
        // Enter・他の場所のクリックで確定、Escは元の名前に戻るので変化なしになる
        const bool done = ImGui::IsItemDeactivated();
        if(done && ImGui::IsItemDeactivatedAfterEdit() && name_[0]) {
          LayerProps p = props;
          p.name       = name_;
          st.set_layer_props(i, p);
        }
        if(done) rename_.reset();
        focus_rename_ = false;
      } else {
        const ImVec2 pos = ImGui::GetCursorScreenPos();
        const float w    = ImGui::GetContentRegionAvail().x;
        if(ImGui::Selectable("##row", i == st.active_layer(), ImGuiSelectableFlags_AllowDoubleClick, ImVec2(0, ImGui::GetFrameHeight()))) {
          st.set_active_layer(i);
          if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            rename_ = Target{doc->handle(), i};
            std::snprintf(name_, sizeof(name_), "%s", props.name.c_str());
            focus_rename_ = true;
          }
        }
        if(ImGui::BeginDragDropSource()) {
          ImGui::SetDragDropPayload("PAINT_LAYER", &i, sizeof(i));
          ImGui::TextUnformatted(props.name.c_str());
          ImGui::EndDragDropSource();
        }
        if(ImGui::BeginDragDropTarget()) {
          if(const auto* payload = ImGui::AcceptDragDropPayload("PAINT_LAYER")) {
            move_from = *static_cast<const int*>(payload->Data);
            move_to   = i;
          }
          ImGui::EndDragDropTarget();
        }
        // 名前(左)と、不透明度・合成モード(右)。名前が長ければ右の情報の手前で切る
        char info[64];
        std::snprintf(info, sizeof(info), "%d%% %s", percent(props.opacity), blend_name(props.blend));
        auto* dl           = ImGui::GetWindowDrawList();
        const float y      = pos.y + ImGui::GetStyle().FramePadding.y;
        const float info_x = pos.x + w - ImGui::CalcTextSize(info).x;
        const ImVec4 clip(pos.x, pos.y, std::max(pos.x, info_x - 6), pos.y + ImGui::GetFrameHeight());
        dl->AddText(nullptr, 0.0f, ImVec2(pos.x, y), ImGui::GetColorU32(ImGuiCol_Text), props.name.c_str(), nullptr, 0.0f, &clip);
        dl->AddText(ImVec2(info_x, y), ImGui::GetColorU32(ImGuiCol_TextDisabled), info);
      }
      ImGui::PopID();
    }
  }
  ImGui::EndChild();
  if(move_from >= 0 && move_from < (int)doc->layers.size() && move_from != move_to) {
    st.set_active_layer(move_from);
    st.move_layer(move_to);
  }
  ImGui::End();
}

// U5で効果や表現色を足すまでは、選択中レイヤーの情報を出すだけ
void LayerPropertyPanel::Update() {
  if(ImGui::Begin(paint_window_title("layer_prop").c_str(), &open)) {
    auto& st                 = paint_editor();
    const PaintDocument* doc = st.doc();
    if(doc && !doc->layers.empty()) {
      const PaintLayer& L = doc->layers[st.active_layer()];
      ImGui::TextUnformatted(L.name.c_str());
      ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
      ImGui::TextWrapped("%s / 不透明度 %d%%", blend_name(L.blend), percent(L.opacity));
      ImGui::PopStyleColor();
    } else {
      ImGui::TextDisabled("キャンバスがありません");
    }
  }
  ImGui::End();
}

} // namespace mu
