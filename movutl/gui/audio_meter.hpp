#pragma once
#include <imgui.h>

namespace mu {

class Composition;

// 現在フレーム付近のミックス後音声(直近約1024サンプル)のレベル。値は0-1(フルスケール=1)
struct AudioLevels {
  float rms[2]  = {0, 0};
  float peak[2] = {0, 0};
};

// 音声バッファが無ければfalse。結果はlevelsへ書く
bool audio_current_levels(Composition* comp, AudioLevels& levels);

// 現在フレーム付近の左chの波形をmin..maxの矩形へ描く(Viewerの操作行用)
void draw_audio_wave(ImDrawList* dl, const ImVec2& min, const ImVec2& max, Composition* comp);

// L/Rメーターとマスター音量フェーダーをウィンドウ右端に描く。BeginTimeline()がInnerClipRect幅を使うため、生成時にそれを狭め破棄時に戻す
class TimelineRightStrip {
  void* win_         = nullptr; // ImGuiWindow*(imgui_internal.hを公開しないためvoid*)
  float saved_max_x_ = 0;

public:
  static constexpr float kWidth = 64.0f;
  explicit TimelineRightStrip(Composition* comp);
  ~TimelineRightStrip();
  TimelineRightStrip(const TimelineRightStrip&)            = delete;
  TimelineRightStrip& operator=(const TimelineRightStrip&) = delete;
};

} // namespace mu
