#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cstdio>
#include <imgui.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/midi.hpp>
#include <movutl/gui/piano_roll.hpp>
#include <movutl/gui/vst_edit_ui.hpp>
#include <movutl/plugin/vst/vst_host.hpp>

namespace mu {

namespace {
constexpr int kMinPitch = 21;  // A0
constexpr int kMaxPitch = 108; // C8
constexpr float kRowH   = 14.0f;
constexpr float kKeysW  = 48.0f;

// ponytail: 単一ウィンドウ限定のドラッグ状態(timeline.cppのctx_と同様の静的状態パターン)
struct DragState {
  MidiEntt* target            = nullptr;
  int note_index              = -1;
  int mode                    = 0; // 1=移動, 2=右端リサイズ
  int64_t grab_offset_samples = 0;
};
DragState drag_;

float px_per_sec_ = 80.0f;

bool is_black_key(int pitch) {
  int m = pitch % 12;
  return m == 1 || m == 3 || m == 6 || m == 8 || m == 10;
}

const char* pitch_name(int pitch) {
  static const char* kNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  static char buf[8];
  std::snprintf(buf, sizeof(buf), "%s%d", kNames[pitch % 12], pitch / 12 - 1);
  return buf;
}
} // namespace

void PianoRollWindow::Update() {
  ImGui::Begin(ICON_FA_KEYBOARD " ピアノロール");

  MidiEntt* midi = nullptr;
  for(auto& e : get_selected_entts()) {
    if(e->getType() == EntityType_Midi) {
      midi = static_cast<MidiEntt*>(e.get());
      break;
    }
  }
  if(!midi) {
    ImGui::TextDisabled("MIDIトラックが選択されていません");
    ImGui::End();
    return;
  }

  { // 音源選択 + Edit(ネイティブGUI/汎用パラメータ一覧)
    auto plugins          = vst_host::plugin_list();
    std::string cur_label = midi->instrument_plugin_id_.empty() ? "(未選択)" : midi->instrument_plugin_id_;
    for(auto& p : plugins) {
      if(p.id == midi->instrument_plugin_id_) cur_label = p.name;
    }
    ImGui::SetNextItemWidth(240);
    if(ImGui::BeginCombo("音源", cur_label.c_str())) {
      for(auto& p : plugins) {
        if(ImGui::Selectable(p.name.c_str(), p.id == midi->instrument_plugin_id_)) midi->assign_instrument(p.id);
      }
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    draw_vst_edit_button("piano_roll_instrument_edit", vst_host::get_instance(midi->instrument_instance_id()));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    ImGui::SliderFloat("倍率", &px_per_sec_, 10.0f, 400.0f, "%.0f px/s");
  }

  auto* comp                = midi->get_comp();
  int sr                    = comp ? comp->audio_sample_rate : 48000;
  int64_t track_len_samples = comp ? (comp->frame_to_sample(midi->fend_) - comp->frame_to_sample(midi->fstart_)) : (int64_t)sr * 8;
  track_len_samples         = std::max<int64_t>(track_len_samples, (int64_t)sr * 4); // 最低4秒分は表示する

  float content_w = kKeysW + (float)track_len_samples / sr * px_per_sec_ + 40.0f;
  float content_h = (float)(kMaxPitch - kMinPitch + 1) * kRowH;

  ImGui::BeginChild("piano_roll_canvas", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);
  ImVec2 origin = ImGui::GetCursorScreenPos();
  ImGui::Dummy(ImVec2(content_w, content_h)); // スクロール範囲を確保するためのダミー
  auto* dl = ImGui::GetWindowDrawList();

  auto pitch_to_y  = [&](int pitch) { return origin.y + (float)(kMaxPitch - pitch) * kRowH; };
  auto sample_to_x = [&](int64_t s) { return origin.x + kKeysW + (float)s / sr * px_per_sec_; };
  auto x_to_sample = [&](float x) -> int64_t { return (int64_t)((x - origin.x - kKeysW) / px_per_sec_ * sr); };
  auto y_to_pitch  = [&](float y) -> int { return kMaxPitch - (int)((y - origin.y) / kRowH); };

  for(int p = kMinPitch; p <= kMaxPitch; p++) {
    float y = pitch_to_y(p);
    dl->AddRectFilled(ImVec2(origin.x + kKeysW, y), ImVec2(origin.x + content_w, y + kRowH), is_black_key(p) ? IM_COL32(30, 30, 30, 255) : IM_COL32(45, 45, 45, 255));
    dl->AddRectFilled(ImVec2(origin.x, y), ImVec2(origin.x + kKeysW, y + kRowH), is_black_key(p) ? IM_COL32(20, 20, 20, 255) : IM_COL32(230, 230, 230, 255));
    dl->AddRect(ImVec2(origin.x, y), ImVec2(origin.x + kKeysW, y + kRowH), IM_COL32(80, 80, 80, 255));
    dl->AddText(ImVec2(origin.x + 2, y + 1), is_black_key(p) ? IM_COL32(255, 255, 255, 200) : IM_COL32(20, 20, 20, 200), pitch_name(p));
  }
  for(int sec = 0; sec <= (int)(track_len_samples / sr) + 1; sec++) {
    float x = sample_to_x((int64_t)sec * sr);
    dl->AddLine(ImVec2(x, origin.y), ImVec2(x, origin.y + content_h), IM_COL32(255, 255, 255, 25));
  }

  ImVec2 mouse        = ImGui::GetMousePos();
  bool canvas_hovered = ImGui::IsWindowHovered();

  auto& notes      = midi->notes();
  int hovered_note = -1;
  int hovered_edge = 0; // 2ならリサイズハンドル(ノート右端)
  for(int i = 0; i < (int)notes.size(); i++) {
    auto& note = notes[i];
    float x0   = sample_to_x(note.start_sample);
    float x1   = sample_to_x(note.start_sample + std::max<int64_t>(note.dur_samples, sr / 16));
    float y0   = pitch_to_y(note.pitch);
    float y1   = y0 + kRowH;
    dl->AddRectFilled(ImVec2(x0, y0 + 1), ImVec2(x1, y1 - 1), IM_COL32(100, 200, 255, 230), 2.0f);
    dl->AddRect(ImVec2(x0, y0 + 1), ImVec2(x1, y1 - 1), IM_COL32(20, 60, 100, 255), 2.0f);
    if(canvas_hovered && mouse.y >= y0 && mouse.y < y1 && mouse.x >= x0 && mouse.x < x1) {
      hovered_note = i;
      hovered_edge = (mouse.x >= x1 - 6.0f) ? 2 : 1;
    }
  }

  if(canvas_hovered && drag_.target == nullptr && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    if(hovered_note >= 0) {
      drag_.target              = midi;
      drag_.note_index          = hovered_note;
      drag_.mode                = hovered_edge;
      drag_.grab_offset_samples = x_to_sample(mouse.x) - notes[hovered_note].start_sample;
    } else if(mouse.x > origin.x + kKeysW) {
      MidiNote n;
      n.pitch        = (uint8_t)std::clamp(y_to_pitch(mouse.y), 0, 127);
      n.start_sample = std::max<int64_t>(0, x_to_sample(mouse.x));
      n.dur_samples  = sr / 2; // デフォルト長0.5秒(テンポ管理が無いため固定値)
      n.velocity     = 100;
      notes.push_back(n);
    }
  }
  if(canvas_hovered && hovered_note >= 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) notes.erase(notes.begin() + hovered_note);

  if(drag_.target == midi && drag_.note_index >= 0 && drag_.note_index < (int)notes.size()) {
    if(ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      auto& note = notes[drag_.note_index];
      if(drag_.mode == 2) {
        int64_t new_end  = std::max<int64_t>(x_to_sample(mouse.x), note.start_sample + sr / 32);
        note.dur_samples = new_end - note.start_sample;
      } else {
        note.start_sample = std::max<int64_t>(0, x_to_sample(mouse.x) - drag_.grab_offset_samples);
        note.pitch        = (uint8_t)std::clamp(y_to_pitch(mouse.y), 0, 127);
      }
    }
    if(ImGui::IsMouseReleased(ImGuiMouseButton_Left)) drag_ = DragState{};
  }

  ImGui::EndChild();
  ImGui::End();
}

} // namespace mu
