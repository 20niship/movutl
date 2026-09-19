#pragma once
#include <imgui.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>

namespace mu {
using FrameT = int32_t;
struct ImTimelineColors {
  ImColor bg                = IM_COL32(10, 10, 10, 255);
  ImColor border            = IM_COL32(0, 0, 0, 255);
  ImColor keyframe          = IM_COL32(255, 255, 255, 255);
  ImColor selected_keyframe = IM_COL32(255, 255, 0, 255);
  ImColor cursor            = IM_COL32(255, 0, 0, 255);
  ImColor cursor_label      = IM_COL32(255, 255, 255, 255);
  ImColor header_bg         = IM_COL32(50, 50, 50, 255);
  ImColor reserved_line     = IM_COL32(0, 0, 255, 255);
};
enum ImTimelineFlags {
  ImTimelineFlags_NoShortcutKeys = 1 << 0,
  ImTimelineFlags_NoContextMenu  = 1 << 1,
};

bool BeginTimeline(const char* name, FrameT* frame, FrameT* start, FrameT* end, bool* playing, float fps = 30.0f, const ImVec2& size = ImVec2(0, 0));
int EndTimeline();                               // returns current key
bool BeginLayer(Composition* cp, int layer_idx); // depth = レイヤのインデックス
void EndLayer();
bool BeginTrack(const Ref<Entity>& entity);
void EndTrack();
bool Keyframe(FrameT* frame);
bool IsTimelineKeyHovered();
bool IsTimeline_LineHovered();
bool IsTimelineClickedLeftButton(); // タイムラインのプロパティ名の左側にあるボタンをクリックしたか
void SetTimelineViewRange(FrameT start, FrameT end);
// タイムライン右端に別ウィジェット(音量メーター等)用の幅を空ける。次回のBeginTimelineから有効
void SetTimelineRightStripWidth(int w);
// ヘッダーのフィットアイコンが押されたか(押されていたらtrueを返しフラグをリセットする)
bool ConsumeTimelineFitRequest();
// 次フレームでタイムラインの表示範囲を全Entityへフィットさせる(ヘッダーのフィットアイコンと同じ)
void RequestTimelineFit();
// ヘッダーの検索欄(レイヤー名/エンティティ名フィルタ)の現在の文字列
const char* GetTimelineLayerSearch();

// 「ここに追加」メニュー項目(BeginMenu/BeginPopup内で呼ぶ)。選択されたら次のEndTimelineでframe/layerへ追加する(layer<0なら空きレイヤー)
bool TimelineAddEntityMenu(int frame, int layer);
// スナップの有効フラグ(操作バーのトグル用)
bool* TimelineSnapFlag();
// 現在の表示範囲(フレーム)。有効ならtrue
bool GetTimelineViewRange(FrameT* start, FrameT* end);
// 表示中心を保ったまま表示幅(フレーム数)を設定する
void SetTimelineVisibleFrames(float frames);

void ResetTimelineState();

} // namespace mu
