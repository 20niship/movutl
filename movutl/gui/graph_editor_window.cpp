#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <imgui.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/anim.hpp>
#include <movutl/gui/graph_editor_window.hpp>
#include <movutl/gui/widgets.hpp>
#include <vector>

namespace mu {

namespace {

struct GraphChannel {
  uint64_t entity_guid = 0;
  int filter_index     = -1;
  std::string prop_name;
  ImU32 color = 0;
};

struct GraphEditorState {
  std::vector<GraphChannel> channels;
  int fstart = 0, fend = 300;
};

const ImU32 kChannelPalette[] = {
  IM_COL32(255, 120, 120, 255), IM_COL32(120, 200, 255, 255), IM_COL32(140, 230, 140, 255), IM_COL32(255, 200, 100, 255), IM_COL32(200, 140, 255, 255), IM_COL32(100, 220, 220, 255),
};

Ref<Entity> find_entity_by_guid(uint64_t guid) {
  for(auto& e : Project::Get()->entities)
    if(e && e->guid_ == guid) return e;
  return nullptr;
}

// チャンネルが指すAnimProps(Entity本体 or filters_[filter_index])を取得する
AnimProps* resolve_anim_props(const GraphChannel& ch, Ref<Entity>& out_entity) {
  auto e = find_entity_by_guid(ch.entity_guid);
  if(!e) return nullptr;
  out_entity = e;
  if(ch.filter_index < 0) {
    e->ensure_anim_props();
    return &e->anim_props_;
  }
  if(ch.filter_index >= (int)e->filters_.size()) return nullptr;
  return &e->filters_[ch.filter_index].props;
}

bool is_scalar_type(const cutil::PropInfo* t) {
  return t == cutil::prop_info_of<float>() || t == cutil::prop_info_of<int32_t>() || t == cutil::prop_info_of<bool>() || t == cutil::prop_info_of<Vec2>() || t == cutil::prop_info_of<Vec3>() || t == cutil::prop_info_of<Vec4>() || t == cutil::prop_info_of<Vec4b>();
}

// ベクトル型は先頭成分(x)だけをグラフ表示する(多成分同時編集は非対応)
float get_scalar(AnimProps& anim, int idx, uint32_t frame) {
  const cutil::PropInfo* t = anim.get_type(idx);
  if(t == cutil::prop_info_of<float>()) return anim.get<float>(idx, frame);
  if(t == cutil::prop_info_of<int32_t>()) return (float)anim.get<int>(idx, frame);
  if(t == cutil::prop_info_of<bool>()) return anim.get<bool>(idx, frame) ? 1.0f : 0.0f;
  if(t == cutil::prop_info_of<Vec2>()) return anim.get<Vec2>(idx, frame)[0];
  if(t == cutil::prop_info_of<Vec3>()) return anim.get<Vec3>(idx, frame)[0];
  if(t == cutil::prop_info_of<Vec4>()) return anim.get<Vec4>(idx, frame)[0];
  if(t == cutil::prop_info_of<Vec4b>()) return (float)anim.get<Vec4b>(idx, frame)[0];
  return 0.0f;
}

void set_scalar(AnimProps& anim, int idx, uint32_t frame, float v) {
  const cutil::PropInfo* t = anim.get_type(idx);
  if(t == cutil::prop_info_of<float>()) {
    anim.add_keyframe<float>(idx, frame, v);
  } else if(t == cutil::prop_info_of<int32_t>()) {
    anim.add_keyframe<int>(idx, frame, (int)std::round(v));
  } else if(t == cutil::prop_info_of<bool>()) {
    anim.add_keyframe<bool>(idx, frame, v >= 0.5f);
  } else if(t == cutil::prop_info_of<Vec2>()) {
    Vec2 val = anim.get<Vec2>(idx, frame);
    val[0]   = v;
    anim.add_keyframe<Vec2>(idx, frame, val);
  } else if(t == cutil::prop_info_of<Vec3>()) {
    Vec3 val = anim.get<Vec3>(idx, frame);
    val[0]   = v;
    anim.add_keyframe<Vec3>(idx, frame, val);
  } else if(t == cutil::prop_info_of<Vec4>()) {
    Vec4 val = anim.get<Vec4>(idx, frame);
    val[0]   = v;
    anim.add_keyframe<Vec4>(idx, frame, val);
  } else if(t == cutil::prop_info_of<Vec4b>()) {
    Vec4b val = anim.get<Vec4b>(idx, frame);
    val[0]    = (uint8_t)std::clamp(v, 0.0f, 255.0f);
    anim.add_keyframe<Vec4b>(idx, frame, val);
  }
}

void draw_channel_list(GraphEditorState& state) {
  ImGui::BeginChild("##channel_list", ImVec2(220, 0), true);
  ImGui::TextDisabled("チャンネル");
  ImGui::Separator();
  for(int i = 0; i < (int)state.channels.size(); i++) {
    auto& ch = state.channels[i];
    ImGui::PushID(i);
    Ref<Entity> e;
    auto* anim  = resolve_anim_props(ch, e);
    bool broken = !anim || anim->index_of(ch.prop_name) < 0;
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ch.color), "%s", broken ? "(missing)" : ch.prop_name.c_str());
    if(e && ImGui::IsItemHovered()) ImGui::SetTooltip("%s%s", e->name.c_str(), ch.filter_index >= 0 ? " (filter)" : "");
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 20);
    if(ImGui::SmallButton(ICON_FA_XMARK)) {
      state.channels.erase(state.channels.begin() + i);
      ImGui::PopID();
      i--;
      continue;
    }
    ImGui::PopID();
  }
  ImGui::Dummy(ImVec2(-1, 40));
  ImGui::TextDisabled("(プロパティ行からここへ\nドラッグ&ドロップ)");
  if(ImGui::BeginDragDropTarget()) {
    if(const ImGuiPayload* p = ImGui::AcceptDragDropPayload(kGraphDragDropId)) {
      const auto* payload = (const GraphDragPayload*)p->Data;
      GraphChannel ch;
      ch.entity_guid  = payload->entity_guid;
      ch.filter_index = payload->filter_index;
      ch.prop_name    = payload->prop_name;
      ch.color        = kChannelPalette[state.channels.size() % IM_ARRAYSIZE(kChannelPalette)];
      state.channels.push_back(ch);
    }
    ImGui::EndDragDropTarget();
  }
  ImGui::EndChild();
}

// 選択中キーフレーム(ドラッグ中のハンドル編集用)。graph_editor_window.cpp内のみで完結する状態なのでファイルスコープでよい
struct SelectedKey {
  uint64_t entity_guid = 0;
  int filter_index     = -1;
  std::string prop_name;
  uint32_t frame = 0;
  bool valid     = false;
};
SelectedKey g_selected_key;

void draw_graph_area(GraphEditorState& state, uint32_t cur_frame) {
  ImGui::BeginChild("##graph_area", ImVec2(0, -140), true);
  ImVec2 avail   = ImGui::GetContentRegionAvail();
  avail.x        = std::max(avail.x, 1.0f);
  avail.y        = std::max(avail.y, 1.0f);
  ImVec2 origin  = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), IM_COL32(25, 25, 25, 255));

  const int range = std::max(state.fend - state.fstart, 1);
  auto frame_to_x = [&](int frame) { return origin.x + avail.x * std::clamp((float)(frame - state.fstart) / (float)range, 0.0f, 1.0f); };
  auto x_to_frame = [&](float x) { return state.fstart + (int)std::round((x - origin.x) / avail.x * range); };

  float cx = frame_to_x((int)cur_frame);
  dl->AddLine(ImVec2(cx, origin.y), ImVec2(cx, origin.y + avail.y), IM_COL32(255, 255, 255, 120));

  ImGui::InvisibleButton("##graph_bg", avail);

  for(auto& ch : state.channels) {
    Ref<Entity> e;
    auto* anim = resolve_anim_props(ch, e);
    if(!anim) continue;
    int idx = anim->index_of(ch.prop_name);
    if(idx < 0) continue;
    const cutil::PropInfo* t = anim->get_type(idx);
    if(!is_scalar_type(t)) continue;

    auto frames = anim->keyframe_frames(idx);
    float vmin = 0.0f, vmax = 1.0f;
    if(!frames.empty()) {
      vmin = vmax = get_scalar(*anim, idx, frames[0]);
      for(uint32_t f : frames) {
        float v = get_scalar(*anim, idx, f);
        vmin    = std::min(vmin, v);
        vmax    = std::max(vmax, v);
      }
    }
    if(vmax - vmin < 1e-4f) {
      vmin -= 1.0f;
      vmax += 1.0f;
    }
    float pad = (vmax - vmin) * 0.1f;
    vmin -= pad;
    vmax += pad;
    auto value_to_y = [&](float v) { return origin.y + avail.y * (1.0f - std::clamp((v - vmin) / (vmax - vmin), 0.0f, 1.0f)); };

    // 折れ線サンプリング(1pxごとにget()を呼ぶ。Customベジエ等の補間もget()内で評価済み)
    ImVec2 prev;
    bool has_prev = false;
    for(int x = 0; x <= (int)avail.x; x++) {
      int f   = x_to_frame(origin.x + x);
      float v = get_scalar(*anim, idx, (uint32_t)std::max(f, 0));
      ImVec2 cur(origin.x + x, value_to_y(v));
      if(has_prev) dl->AddLine(prev, cur, ch.color, 2.0f);
      prev     = cur;
      has_prev = true;
    }

    for(int ki = 0; ki < (int)frames.size(); ki++) {
      uint32_t kf = frames[ki];
      if((int)kf < state.fstart || (int)kf > state.fend) continue;
      float v = get_scalar(*anim, idx, kf);
      ImVec2 center(frame_to_x((int)kf), value_to_y(v));
      bool selected = g_selected_key.valid && g_selected_key.entity_guid == ch.entity_guid && g_selected_key.filter_index == ch.filter_index && g_selected_key.prop_name == ch.prop_name && g_selected_key.frame == kf;
      ImU32 col     = selected ? IM_COL32(255, 255, 255, 255) : ch.color;
      dl->AddQuadFilled(ImVec2(center.x, center.y - 5), ImVec2(center.x + 5, center.y), ImVec2(center.x, center.y + 5), ImVec2(center.x - 5, center.y), col);

      ImGui::PushID(ch.prop_name.c_str());
      ImGui::PushID(ch.filter_index);
      ImGui::PushID(ki); // frame/valueはドラッグ中に変化しIDが不安定になるため配列indexを使う
      ImGui::SetCursorScreenPos(ImVec2(center.x - 5, center.y - 5));
      ImGui::InvisibleButton("##kf", ImVec2(10, 10));
      if(ImGui::IsItemActivated()) {
        g_selected_key = SelectedKey{ch.entity_guid, ch.filter_index, ch.prop_name, kf, true};
      }
      if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        ImVec2 mp        = ImGui::GetMousePos();
        int new_frame    = std::clamp(x_to_frame(mp.x), state.fstart, state.fend);
        float new_value  = vmin + (1.0f - std::clamp((mp.y - origin.y) / avail.y, 0.0f, 1.0f)) * (vmax - vmin);
        uint32_t cur_key = g_selected_key.frame;
        if((uint32_t)new_frame != cur_key) anim->move_keyframe(idx, cur_key, (uint32_t)new_frame);
        set_scalar(*anim, idx, (uint32_t)new_frame, new_value);
        g_selected_key.frame = (uint32_t)new_frame;
      }
      if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) anim->erase_keyframe(idx, kf);
      ImGui::PopID();
      ImGui::PopID();
      ImGui::PopID();
    }
  }
  ImGui::EndChild();
}

void draw_selected_key_editor() {
  ImGui::BeginChild("##kf_editor", ImVec2(0, 0), true);
  if(!g_selected_key.valid) {
    ImGui::TextDisabled("キーフレームを選択してください");
    ImGui::EndChild();
    return;
  }
  GraphChannel ch;
  ch.entity_guid  = g_selected_key.entity_guid;
  ch.filter_index = g_selected_key.filter_index;
  ch.prop_name    = g_selected_key.prop_name;
  Ref<Entity> e;
  auto* anim = resolve_anim_props(ch, e);
  int idx    = anim ? anim->index_of(ch.prop_name) : -1;
  if(!anim || idx < 0 || !anim->has_key_at(idx, g_selected_key.frame)) {
    ImGui::TextDisabled("選択中のキーフレームは削除されました");
    ImGui::EndChild();
    return;
  }

  ImGui::Text("%s (frame %u)", ch.prop_name.c_str(), g_selected_key.frame);
  AniInterpType cur = anim->get_ease_type(idx, g_selected_key.frame);
  if(cur == AniInterpType::Custom) {
    auto bez = anim->get_ease_bezier(idx, g_selected_key.frame);
    if(wd_bezier_handle_editor(bez, 100.0f)) anim->set_ease_bezier(idx, g_selected_key.frame, bez);
  } else {
    ImGui::TextDisabled("(ダブルクリックでイージング編集: インスペクタのミニストリップを参照)");
  }
  ImGui::EndChild();
}

} // namespace

void GraphEditorWindow::Update() {
  ImGui::Begin(ICON_FA_CHART_LINE " グラフエディタ", &open);
  static GraphEditorState state;

  auto* comp         = Composition::GetActiveComp();
  uint32_t cur_frame = comp ? (uint32_t)std::max(comp->get_frame(), 0) : 0;
  if(comp) {
    state.fstart = comp->fstart;
    state.fend   = comp->fend;
  }

  draw_channel_list(state);
  ImGui::SameLine();
  ImGui::BeginGroup();
  draw_graph_area(state, cur_frame);
  draw_selected_key_editor();
  ImGui::EndGroup();

  ImGui::End();
}

} // namespace mu
