#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cstring>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/inspector.hpp>
#include <movutl/gui/widgets.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu {

namespace {
// movutl/core/string.hpp 削除に伴い、唯一の利用箇所であるここに移動 ([#14])
bool fuzzy_match(const char* src, const char* filter) {
  if(!src || !filter) return true;
  while(*filter) {
    char c = *filter++;
    src    = std::strchr(src, c);
    if(!src) return false;
    src++;
  }
  return true;
}
} // namespace

void InspectorWindow::Update() {
  ImGui::Begin(ICON_FA_PLUG " エフェクト制御");
  auto entts = get_selected_entts();
  if(entts.empty()) {
    ImGui::TextDisabled("オブジェクトが選択されていません");
    ImGui::End();
    return;
  }
  Ref<Entity> e = entts[0];

  {
    // アクティブ(目アイコン): このEntityの表示/非表示を切り替える(音声はミュートも兼ねる)
    if(ImGui::SmallButton(e->trk.active_ ? ICON_FA_EYE : ICON_FA_EYE_SLASH)) {
      e->trk.active_ = !e->trk.active_;
      if(auto* comp = e->get_comp()) comp->cache.invalidate_all();
    }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip(e->trk.active_ ? "非表示にする" : "表示する");
    ImGui::SameLine();
    const std::string str = get_entt_icon(e) + std::string(" ") + e->name.c_str();
    ImGui::SmallButton(str.c_str());
  }

  { // 合成モード(BlendType): TrackObject側のプロパティのため専用UIとして扱う
    static const char* kBlendNames[] = {"通常", "加算", "減算", "乗算", "除算", "スクリーン", "オーバーレイ", "比較(暗)", "比較(明)", "ハードライト"};
    int idx = std::clamp((int)e->trk.blend_, 0, (int)IM_ARRAYSIZE(kBlendNames) - 1);
    ImGui::SetNextItemWidth(-1);
    if(ImGui::Combo("合成モード", &idx, kBlendNames, IM_ARRAYSIZE(kBlendNames))) {
      e->trk.blend_ = (BlendType)idx;
      if(auto* comp = e->get_comp()) comp->cache.invalidate_all();
    }
  }

  wd_entt_props_editor(e.get());

  for(int i = 0; i < e->trk.filters.size(); i++) {
    auto& f = e->trk.filters[i];
    MU_ASSERT(f.plg_ != nullptr);
    ImGui::PushID(i);
    std::string FX_ICON = ICON_FA_PLUG " ";
    std::string str     = FX_ICON + f.plg_->name.c_str();
    { // チェックボックス風のON/OFF切り替え(クリックでenabledを反転)
      auto c        = ImGui::GetCursorScreenPos();
      auto h        = ImGui::GetTextLineHeight();
      auto dl       = ImGui::GetWindowDrawList();
      ImRect box(c, ImVec2(c.x + h, c.y + h));
      bool hovers = ImGui::IsMouseHoveringRect(box.Min, box.Max);
      dl->AddRectFilled(box.Min, box.Max, hovers ? IM_COL32(255, 255, 255, 45) : IM_COL32(255, 255, 255, 20), 3.0f);
      dl->AddRect(box.Min, box.Max, IM_COL32(255, 255, 255, 120), 3.0f);
      if(f.enabled) dl->AddText(ImVec2(box.Min.x + 2, box.Min.y), IM_COL32(120, 220, 120, 255), ICON_FA_CHECK);
      if(hovers) {
        ImGui::SetTooltip("エフェクト %s を有効/無効にします", f.plg_->name.c_str());
        if(ImGui::IsMouseClicked(0)) {
          f.enabled = !f.enabled;
          if(auto* comp = e->get_comp()) comp->cache.invalidate_all();
        }
      }
      ImGui::Dummy(ImVec2(h + 4, h));
      ImGui::SameLine();
    }
    if(ImGui::TreeNode(str.c_str())) {
      bool props_changed = false;
      int size_          = std::min<int>(f.props.size(), (int)e->trk.filters[i].plg_->props.fields.size());
      for(int k = 0; k < size_; k++) {
        const auto& info = f.plg_->props.fields[k];
        ImGui::PushID(k);
        { // animation props editor
          const char* label = info.label[0] ? info.label : info.name;
          if(f.props.get_type(k) != info.type) {
            LOG_F(ERROR, "Invalid type: %s", info.name);
          } else if(info.type == cutil::prop_info_of<float>()) {
            float value = f.props.get<float>(k);
            if(ImGui::DragFloat(label, &value, info.drag_speed, info.min_value, info.max_value)) {
              f.props.set_value(k, 0, value);
              props_changed = true;
            }
          } else if(info.type == cutil::prop_info_of<int32_t>()) {
            int value = f.props.get<int>(k);
            if(ImGui::DragInt(label, &value, info.drag_speed)) {
              f.props.set_value(k, 0, value);
              props_changed = true;
            }
          } else if(info.type == cutil::prop_info_of<std::string>()) {
            std::string value = f.props.get<std::string>(k);
            static char buf[256];
            strncpy(buf, value.c_str(), 256);
            if(ImGui::InputText(label, &buf[0], 256)) {
              f.props.set_value(k, 0, std::string(buf));
              props_changed = true;
            }
          } else if(info.type == cutil::prop_info_of<bool>()) {
            bool value = f.props.get<bool>(k);
            if(ImGui::Checkbox(label, &value)) {
              f.props.set_value(k, 0, value);
              props_changed = true;
            }
          } else if(info.type == cutil::prop_info_of<Vec2>()) {
            Vec2 value = f.props.get<Vec2>(k);
            if(ImGui::DragFloat2(label, value.value, info.drag_speed)) {
              f.props.set_value(k, 0, value);
              props_changed = true;
            }
          } else if(info.type == cutil::prop_info_of<Vec3>()) {
            Vec3 value = f.props.get<Vec3>(k);
            if(ImGui::DragFloat3(label, value.value, info.drag_speed)) {
              f.props.set_value(k, 0, value);
              props_changed = true;
            }
          } else if(info.type == cutil::prop_info_of<Vec4>()) {
            Vec4 value = f.props.get<Vec4>(k);
            if(ImGui::DragFloat4(label, value.value, info.drag_speed)) {
              f.props.set_value(k, 0, value);
              props_changed = true;
            }
          }
        }
        ImGui::PopID();
      }
      if(props_changed) {
        if(auto* comp = e->get_comp()) comp->cache.invalidate_all();
      }
      ImGui::TreePop();
    }
    ImGui::PopID();
  }

  {
    static bool open_popup         = false;
    static bool focus_search       = false;
    static char search_buffer[64]  = "";

    // 「Add Filter」ボタン
    if(ImGui::Button("フィルタを追加する")) {
      open_popup    = true;
      focus_search  = true;
      ImGui::OpenPopup("##INSPECTOR_FILTER_POPUP");
    }
    if(open_popup && ImGui::BeginPopup("##INSPECTOR_FILTER_POPUP")) {
      ImGui::Text("エフェクトを追加する");
      ImGui::Separator();

      if(focus_search) {
        ImGui::SetKeyboardFocusHere();
        focus_search = false;
      }
      ImGui::InputText("Search", search_buffer, IM_ARRAYSIZE(search_buffer));
      auto filters       = &detail::AppMain::Get()->filters;
      bool is_audio_entt = e->getType() == EntityType_Audio;
      for(int i = 0; i < filters->size(); i++) {
        // 音声専用フィルタを音声トラック以外に付けると描画スレッドで音声処理関数が呼ばれ落ちるため出し分ける
        if(((*filters)[i].flag == FilterAudioOnly) != is_audio_entt) continue;
        const char* name = (*filters)[i].name.c_str();
        if(!fuzzy_match(name, search_buffer)) continue;
        if(ImGui::Selectable(name)) {
          TrackObject::FilterParam fp;
          fp.plg_ = &(*filters)[i];
          fp.props.add_props((*filters)[i].defaults);
          fp.enabled = true;
          e->trk.filters.push_back(fp);
        }
      }
      ImGui::EndPopup();
    }
  }

  ImGui::End();
}

} // namespace mu
