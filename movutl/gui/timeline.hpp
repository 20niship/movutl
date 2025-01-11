#pragma once
#include <imgui.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/track.hpp>

namespace mu {
using FrameT = int32_t;
struct ImTimelineColors {
  ImColor bg = IM_COL32(10, 10, 10, 255);
  ImColor border = IM_COL32(0, 0, 0, 255);
  ImColor keyframe = IM_COL32(255, 255, 255, 255);
  ImColor selected_keyframe = IM_COL32(255, 255, 0, 255);
  ImColor cursor = IM_COL32(255, 0, 0, 255);
  ImColor cursor_label = IM_COL32(255, 255, 255, 255);
  ImColor header_bg = IM_COL32(50, 50, 50, 255);
  ImColor reserved_line = IM_COL32(0, 0, 255, 255);
};
enum ImTimelineFlags {
  ImTimelineFlags_NoShortcutKeys = 1 << 0,
  ImTimelineFlags_NoContextMenu = 1 << 1,
};

bool BeginTimeline(const char* name, FrameT* frame, FrameT* start, FrameT* end, bool* playing, const ImVec2& size = ImVec2(0, 0));
int EndTimeline();                  // returns current key
bool BeginLayer(TrackLayer* layer); // depth = レイヤのインデックス
void EndLayer();
bool BeginTrack(const Ref<Entity>& entity);
void EndTrack();
bool Keyframe(FrameT* frame);
bool IsTimelineKeyHovered();
bool IsTimeline_LineHovered();
bool IsTimelineClickedLeftButton(); // タイムラインのプロパティ名の左側にあるボタンをクリックしたか
void SetTimelineViewRange(FrameT start, FrameT end);

bool ShouldHideTimelineTabBar();
void ResetTimelineState();

} // namespace mu
