#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cfloat>
#include <cstring>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/compo_audio_ref.hpp>
#include <movutl/asset/compo_ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/asset/entity.hpp>
#ifdef MOVUTL_DAW
#include <movutl/asset/midi.hpp>
#endif
#include <movutl/asset/project.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/graph_editor_window.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/inspector.hpp>
#include <movutl/gui/widgets.hpp>
#include <movutl/plugin/plugin.hpp>
#ifdef MOVUTL_DAW
#include <movutl/gui/vst_edit_ui.hpp>
#include <movutl/plugin/vst/vst_filter_bridge.hpp>
#include <movutl/plugin/vst/vst_host.hpp>
#endif
#include <string>
#include <vector>

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

// テキストの揃え位置(3x3)と文字装飾。数値プロパティ(サイズ/太字/字間など)は汎用のプロパティ欄が担当する
bool draw_text_style_ui(TextEntt* t) {
  bool changed                       = false;
  static const char* kAlignLabels[9] = {"左上", "上", "右上", "左", "中央", "右", "左下", "下", "右下"};
  if(!wd_table_begin("##text_style")) return false;
  wd_row("揃え", "文字ブロックのどの点を位置(基点)に合わせるか");
  int picked = -1;
  if(wd_grid9("##text_align", std::clamp((int)t->align_, 0, 8), &picked, kAlignLabels)) {
    t->align_ = picked;
    changed   = true;
  }
  static const char* kDecoNames[] = {"標準", "影付き", "影付き(薄)", "縁取り", "縁取り(細)"};
  int deco                        = std::clamp((int)t->deco_, 0, (int)IM_ARRAYSIZE(kDecoNames) - 1);
  wd_row("文字装飾");
  if(ImGui::Combo("##deco", &deco, kDecoNames, IM_ARRAYSIZE(kDecoNames))) {
    t->deco_ = deco;
    changed  = true;
  }
  wd_table_end();
  return changed;
}

} // namespace

void InspectorWindow::Update() {
  ImGui::Begin(ICON_FA_PLUG " エフェクト制御", &open);
  auto entts = get_selected_entts();
  if(entts.empty()) {
    ImGui::TextDisabled("オブジェクトが選択されていません");
    ImGui::End();
    return;
  }
  Ref<Entity> e   = entts[0];
  auto invalidate = [&]() {
    if(auto* comp = e->get_comp()) comp->invalidate_cache_range(e->fstart_, e->fend_);
  };
  auto get_cur_frame = [&]() -> uint32_t {
    Composition* comp = e->get_comp();
    return (uint32_t)(comp ? std::max(comp->get_frame(), 0) : 0);
  };
  uint32_t cur_frame = get_cur_frame();
  if(wd_entity_keyframe_overview(e.get(), cur_frame)) cur_frame = get_cur_frame(); // Entity全体の中間点分布バー(クリック/ドラッグでシーク)

  {
    // アクティブ(目アイコン): このEntityの表示/非表示を切り替える(音声はミュートも兼ねる)
    if(ImGui::SmallButton(e->active_ ? ICON_FA_EYE : ICON_FA_EYE_SLASH)) {
      e->active_ = !e->active_;
      invalidate();
    }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip(e->active_ ? "非表示にする" : "表示する");
    ImGui::SameLine();
    const std::string str = std::string(e->clipping_up_ ? ICON_FA_CROP_SIMPLE " " : "") + (e->camera_ctrl_ ? ICON_FA_VIDEO " " : "") + std::string(get_entt_icon(e)) + " " + e->name.c_str();
    ImGui::TextUnformatted(str.c_str());
  }

  if(e->has_transform()) wd_entt_transform_editor(e.get(), cur_frame);

  {
    if(wd_table_begin("##obj_common")) { // 合成モード(BlendType): Entityのトラック共通属性のため専用UIとして扱う
      static const char* kBlendNames[] = {"通常", "加算", "減算", "乗算", "除算", "スクリーン", "オーバーレイ", "比較(暗)", "比較(明)", "ハードライト"};
      int idx                          = std::clamp((int)e->blend_, 0, (int)IM_ARRAYSIZE(kBlendNames) - 1);
      wd_row("合成モード");
      if(ImGui::Combo("##blend", &idx, kBlendNames, IM_ARRAYSIZE(kBlendNames))) {
        e->blend_ = (BlendType)idx;
        invalidate();
      }
      wd_row("上のオブジェクトでクリッピング");
      if(ImGui::Checkbox("##clip_up", &e->clipping_up_)) invalidate();
      wd_row("カメラ制御の対象");
      if(ImGui::Checkbox("##cam_ctrl", &e->camera_ctrl_)) invalidate();
      wd_table_end();
    }

    if(auto* txt = dynamic_cast<TextEntt*>(e.get())) {
      if(draw_text_style_ui(txt)) invalidate();
    }

    if((e->getType() == EntityType_Scene || e->getType() == EntityType_SceneAudio) && wd_table_begin("##compo_ref")) {
      auto pj                = Project::Get();
      uint32_t* target_guid  = (e->getType() == EntityType_Scene) ? &static_cast<CompoRefEntt*>(e.get())->target_comp_guid : &static_cast<CompoAudioEntt*>(e.get())->target_comp_guid;
      Composition* self_comp = e->get_comp();
      std::vector<Composition*> candidates;
      int cur_idx          = -1;
      std::string cur_name = "(未選択)";
      for(auto& c : pj->compos_) {
        if(self_comp && c->guid == self_comp->guid) continue; // 自己参照防止(間接循環はPushRenderGuardで防ぐ)
        if(*target_guid == c->guid) {
          cur_idx  = (int)candidates.size();
          cur_name = c->name.c_str();
        }
        candidates.push_back(c.get());
      }
      wd_row("参照コンポジション");
      if(ImGui::BeginCombo("##compo_ref_combo", cur_name.c_str())) {
        for(int i = 0; i < (int)candidates.size(); i++) {
          bool selected = i == cur_idx;
          if(ImGui::Selectable(candidates[i]->name.c_str(), selected)) {
            *target_guid = candidates[i]->guid;
            if(auto* self_comp2 = e->get_comp()) self_comp2->invalidate_cache_all();
          }
        }
        ImGui::EndCombo();
      }
      wd_table_end();
    }

#ifdef MOVUTL_DAW
    if(e->getType() == EntityType_Midi && wd_table_begin("##midi_inst")) { // 音源選択(vst_host::plugin_list())+ Edit導線(vst_edit_ui)
      auto* midi            = static_cast<MidiEntt*>(e.get());
      auto plugins          = vst_host::plugin_list();
      std::string cur_label = midi->instrument_plugin_id_.empty() ? "(未選択)" : midi->instrument_plugin_id_;
      for(auto& p : plugins) {
        if(p.id == midi->instrument_plugin_id_) cur_label = p.name;
      }
      wd_row("音源プラグイン");
      if(ImGui::BeginCombo("##midi_plugin", cur_label.c_str())) {
        for(auto& p : plugins) {
          if(ImGui::Selectable(p.name.c_str(), p.id == midi->instrument_plugin_id_)) midi->assign_instrument(p.id);
        }
        ImGui::EndCombo();
      }
      wd_row("");
      draw_vst_edit_button("midi_instrument_edit", vst_host::get_instance(midi->instrument_instance_id()));
      wd_table_end();
    }
#endif

    wd_entt_props_editor(e.get(), cur_frame);

    // カスタムオブジェクト(Luaスクリプト)のtrack0-3/check0-3相当のパラメータはgetPropsInfo()を持たない(動的なcutil::Propで保持している)ため専用UIで編集する
    if(auto* custom = dynamic_cast<CustomObjectEntt*>(e.get())) {
      if(const auto* def = custom->def()) {
        bool params_changed = false;
        if(wd_table_begin("##custom_params")) {
          for(const auto& tr : def->tracks) {
            float v = cutil::get_or<float>(custom->params_, tr.name.c_str(), tr.default_value);
            ImGui::PushID(tr.name.c_str());
            wd_row(tr.name.c_str());
            if(ImGui::DragFloat("##v", &v, tr.step, tr.min_value, tr.max_value)) {
              custom->params_.set<float>(tr.name.c_str(), v);
              params_changed = true;
            }
            ImGui::PopID();
          }
          for(const auto& ch : def->checks) {
            bool v = cutil::get_or<bool>(custom->params_, ch.name.c_str(), ch.default_value);
            ImGui::PushID(ch.name.c_str());
            wd_row(ch.name.c_str());
            if(ImGui::Checkbox("##v", &v)) {
              custom->params_.set<bool>(ch.name.c_str(), v);
              params_changed = true;
            }
            ImGui::PopID();
          }
          wd_table_end();
        }
        if(params_changed) invalidate();
      } else {
        ImGui::TextDisabled("スクリプト '%s' が見つかりません", custom->script_name_.c_str());
      }
    }
  }

  // エフェクト: 見出し右端の「＋」とセクション末尾のボタンの両方から追加できる(下にスクロールしないと見つからない問題の対策)
  static char search_buffer[64] = "";
  const std::string fx_title    = std::string("エフェクト (") + std::to_string(e->filters_.size()) + ")###fx_section";
  const bool fx_open            = ImGui::CollapsingHeader(fx_title.c_str(), ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_AllowOverlap);
  {
    const float bw = ImGui::GetFrameHeight();
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - bw - 2.0f);
    if(ImGui::SmallButton(ICON_FA_PLUS "##fx_add_top")) {
      search_buffer[0] = '\0';
      ImGui::OpenPopup("##INSPECTOR_FILTER_POPUP");
    }
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("エフェクトを追加する");
  }

  int remove_idx = -1;
  if(fx_open) {
    for(int i = 0; i < e->filters_.size(); i++) {
      auto& f = e->filters_[i];
      MU_ASSERT(f.plg_ != nullptr);
      ImGui::PushID(i);
      ImGui::Spacing();
      // カード見出し: [有効チェック] [アイコン+名前(折りたたみ)] ...... [VST編集] [削除]
      if(ImGui::Checkbox("##fx_enabled", &f.enabled)) invalidate();
      if(ImGui::IsItemHovered()) ImGui::SetTooltip("エフェクト %s を有効/無効にします", f.plg_->name.c_str());
      ImGui::SameLine();
      const std::string str = std::string(ICON_FA_PLUG " ") + f.plg_->name.c_str() + "###fx_" + std::to_string(i);
      if(!f.enabled) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
      const bool card_open = ImGui::TreeNodeEx(str.c_str(), ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_AllowOverlap);
      if(!f.enabled) ImGui::PopStyleColor();
      {
        const float bw = ImGui::GetFrameHeight();
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - bw - 2.0f);
        if(ImGui::SmallButton(ICON_FA_TRASH "##fx_del")) remove_idx = i;
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("エフェクトを削除");
      }
#ifdef MOVUTL_DAW
      if(detail::is_vst_filter_guid(f.plg_->guid)) {
        ImGui::SameLine();
        draw_vst_edit_button("vst_fx_edit", detail::vst_filter_instance(f.instance_state));
      }
#endif
      if(card_open) {
        bool props_changed = false;
        int size_          = std::min<int>(f.props.size(), (int)e->filters_[i].plg_->props.fields.size());
        // アニメーション可能な型は「左右=直前/直後の中間点値+中央=名前ボタン」のトラックバー行(フル幅)、文字列は「ラベル左・値右」の表で編集する
        for(int k = 0; k < size_; k++) {
          const auto& info = f.plg_->props.fields[k];
          if(cutil::has_flag(info.flags, cutil::PropFlags::Hidden)) continue;
          ImGui::PushID(k);
          bool value_changed = false;
          if(f.props.get_type(k) != info.type) {
            LOG_F(ERROR, "Invalid type: %s", info.name);
          } else if(info.type == cutil::prop_info_of<std::string>()) {
            if(wd_table_begin("##fx_str")) {
              wd_row(info.label[0] ? info.label : info.name, info.desc);
              std::string value = f.props.get<std::string>(k);
              static char buf[256];
              strncpy(buf, value.c_str(), 256);
              if(ImGui::InputText("##v", &buf[0], 256)) {
                f.props.set_value(k, e->rel_frame((int)cur_frame), std::string(buf));
                value_changed = true;
              }
              wd_table_end();
            }
          } else if(info.type == cutil::prop_info_of<float>() || info.type == cutil::prop_info_of<int32_t>() || info.type == cutil::prop_info_of<bool>() || info.type == cutil::prop_info_of<Vec2>() || info.type == cutil::prop_info_of<Vec3>() || info.type == cutil::prop_info_of<Vec4>() ||
                    info.type == cutil::prop_info_of<Vec4b>()) {
            if(wd_animatable_row(info, f.props, k, e->rel_frame((int)cur_frame), e->guid_, i, e->fend_ - e->fstart_)) value_changed = true;
          }
          if(value_changed) props_changed = true;
          ImGui::PopID();
        }
        if(props_changed) invalidate();
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    if(remove_idx >= 0) {
      {
        std::lock_guard<std::mutex> lock(e->mtx);
        e->filters_.erase(e->filters_.begin() + remove_idx);
      }
      invalidate();
    }

    ImGui::Spacing();
    if(ImGui::Button(ICON_FA_PLUS " エフェクトを追加", ImVec2(-FLT_MIN, 0))) {
      search_buffer[0] = '\0';
      ImGui::OpenPopup("##INSPECTOR_FILTER_POPUP");
    }
  }

  // 追加ポップアップ(見出しの＋ボタンと末尾ボタンで共用。IDスコープを揃えるためセクションの外側で開く)
  if(ImGui::BeginPopup("##INSPECTOR_FILTER_POPUP")) {
    ImGui::TextUnformatted("エフェクトを追加する");
    ImGui::Separator();
    if(ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
    ImGui::SetNextItemWidth(240.0f);
    ImGui::InputTextWithHint("##fx_search", ICON_FA_MAGNIFYING_GLASS " 検索", search_buffer, IM_ARRAYSIZE(search_buffer));
    ImGui::BeginChild("##fx_list", ImVec2(240.0f, 260.0f), ImGuiChildFlags_None);
    auto filters       = &detail::AppMain::Get()->filters;
    bool is_audio_entt = e->getType() == EntityType_Audio;
    for(int i = 0; i < filters->size(); i++) {
      // 音声専用フィルタを音声トラック以外に付けると描画スレッドで音声処理関数が呼ばれ落ちるため出し分ける
      if(((*filters)[i].flag == FilterAudioOnly) != is_audio_entt) continue;
      const char* name = (*filters)[i].name.c_str();
      if(!fuzzy_match(name, search_buffer)) continue;
      ImGui::PushID(i);
      bool clicked = ImGui::Selectable(name);
      ImGui::PopID();
      if(clicked) {
        FilterParam fp;
        fp.plg_ = &(*filters)[i];
        fp.props.add_props((*filters)[i].defaults);
        fp.enabled = true;
        e->filters_.push_back(fp);
        invalidate();
        ImGui::CloseCurrentPopup();
      }
    }
    ImGui::EndChild();
    ImGui::EndPopup();
  }

  ImGui::End();
}

} // namespace mu
