#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif

#include <IconsFontAwesome6.h>
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cutil/rect.hpp>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/app/export_state.hpp>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/group.hpp>
#include <movutl/core/command.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/timeline.hpp>
#include <string>
#include <tuple>
#include <vector>

enum ImTimelineState {
  None,
  Selecting,
  Draginctx_Start,
  Draginctx_End,
  Draginctx_Cursor,
  Draginctx_Keyframes,
  Draginctx_Tracks_Start,
  Draginctx_Tracks_End,
  Draginctx_Tracks,
};

namespace mu {

struct TimelineContext {
  cutil::Rect all_area;
  int hidx              = 0;
  int trackname_width   = 140;
  bool toggle_play      = false;
  int height            = 16;
  int lasy_mouse_x      = 0;
  Entity* last_entt_hov = nullptr;
  int header_h          = 20;
  int vis_start         = -10;
  int vis_end           = 100;
  int right_strip_w     = 0;     // タイムライン右端に別ウィジェット(音量メーター等)を置くために空ける幅。SetTimelineRightStripWidthで変更する
  bool ruler_timecode   = false; // 定規の表記(false=フレーム, true=タイムコード)
  float fps             = 30.0f;
  float row_scale       = 1.0f; // レイヤー行の高さ倍率(S/M/L)
  int cur_frame         = 0;
  bool first            = true;
  bool cur_layer_active = true;  // BeginLayerで設定し、そのレイヤー内のBeginTrackが参照する
  std::vector<Entity*> selected; // フレーム冒頭にget_selected_entts()から作る選択キャッシュ(クリック処理で随時更新)

  // 1フレーム内の状態(BeginTimelineでリセット)
  FrameT* frame_ptr     = nullptr; // 呼び出し元のプレイヘッド変数(右クリックメニューからの移動に使う)
  FrameT* start_ptr     = nullptr;
  FrameT* end_ptr       = nullptr;
  bool any_clip_hovered = false;
  int snap_line         = INT_MIN; // スナップ中の縦ガイド線(フレーム)
  struct ClipRect {
    Ref<Entity> e;
    ImRect r;
  };
  std::vector<ClipRect> clips;

  // スナップ/ラバーバンド/コンポ範囲ハンドル
  bool snap = true;
  std::vector<int> snap_points; // ドラッグ開始時に集める他クリップの端
  bool rb_active = false;
  bool rb_moved  = false;
  ImVec2 rb_start;
  int comp_drag      = 0; // 0=なし 1=開始 2=終了
  bool name_col_drag = false;

  // ソロ: 押す前の各レイヤーactiveを保持(UI状態のみ)。solo_layer<0なら非ソロ
  int solo_layer = -1;
  std::vector<bool> solo_saved;

  // 右クリックメニュー
  Ref<Entity> ctx_entt;
  int ctx_frame = 0;
  int ctx_layer = -1;

  // 破壊的操作はEndTimeline()で遅延適用する(BeginTrackはlayer.entts[]への参照を持ったまま呼ばれるため、その最中にvectorを変更しない)
  int pending_clip_op      = 0;  // 1=分割 2=複製 3=削除 4=有効/無効切替 5=上のオブジェクトでクリッピング切替 6=カメラ制御の対象切替
  int pending_insert_layer = -1; // 挿入位置(この位置に空レイヤーを追加)
  struct PendingAdd {
    bool valid      = false;
    EntityType type = EntityType_3DText;
    int shape       = -1; // >=0なら図形(ShapeType)
    int frame       = 0;
    int layer       = -1;
  } pending_add;

  // レイヤー名インライン編集
  int editing_layer_idx      = -1;
  char editing_layer_buf[64] = {0};

  // ヘッダー左端(フィット/タイムコード/検索)
  bool pending_fit          = false;
  bool search_open          = false;
  char layer_search_buf[64] = {0};

  // レイヤーの削除/移動は破壊的操作のためEndTimeline()側で遅延適用する
  Composition* active_comp  = nullptr;
  int pending_delete_layer  = -1;
  int pending_move_layer    = -1;
  Entity* pending_entt_move = nullptr; // ドラッグでレイヤー移動するEntity(フレーム末尾で適用)
  int pending_entt_layer    = -1;
  int pending_move_dir      = 0; // -1: 上へ, +1: 下へ

  // クリップのドラッグ移動/端リサイズ
  Entity* dragging_entt = nullptr;
  int drag_mode         = 0; // 0=none, 1=move, 2=resize_left, 3=resize_right
  int drag_orig_fstart  = 0;
  std::vector<std::tuple<Entity*, int, int>> drag_group_orig; // group_guidが同じ他エンティティの(entity, orig_fstart, orig_fend)。移動ドラッグ開始時のみ使う
  int drag_orig_fend   = 0;
  int drag_start_frame = 0;

  // 集約キーフレーム行のドラッグ移動(BeginTrack下端のダイヤ)。dragging_entt/drag_modeとは別状態にして衝突を避ける
  Entity* dragging_kf_entt    = nullptr;
  uint32_t drag_kf_orig_frame = 0;

  cutil::Rect tl_area() {
    auto r = all_area;
    r.y.min += header_h;
    r.x.min += trackname_width;
    return r;
  }

  cutil::Rect header_area() {
    auto r  = all_area;
    r.y.max = r.y.min + header_h;
    return r;
  }

  int f2view(FrameT f) {
    if(vis_start >= vis_end) return vis_start;
    auto d = f - vis_start;
    return tl_area().x.min + tl_area().w() * d / (vis_end - vis_start);
  }
  int view2f(int x) {
    auto d = x - tl_area().x.min;
    return vis_start + (vis_end - vis_start) * d / tl_area().w();
  }

  int tl_w() { return (int)tl_area().w(); }

  int layer_y1() const { return all_area.y.min + header_h + hidx * height; }
  int layer_y2() const { return all_area.y.min + header_h + (hidx + 1) * height; }
};

static TimelineContext ctx_;
ImTimelineColors col_;


// フレーム番号をAE風のタイムコード(HH:MM:SS:FF)へ変換する
static std::string frame_to_timecode(int frame, float fps) {
  if(fps <= 0.0f) fps = 30.0f;
  int fps_i        = std::max(1, (int)std::round(fps));
  int total_frames = std::max(0, frame);
  int ff           = total_frames % fps_i;
  int total_sec    = total_frames / fps_i;
  int ss           = total_sec % 60;
  int mm           = (total_sec / 60) % 60;
  int hh           = total_sec / 3600;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d:%02d", hh, mm, ss, ff);
  return buf;
}


// 定規の目盛り(主/副)をズームに応じて選ぶ。主目盛りのラベル同士がmin_label_px以上離れる最小の刻みを採る
struct RulerTicks {
  int major = 10;
  int minor = 0; // 0なら副目盛りなし
};

static RulerTicks choose_ruler_ticks(float px_per_frame, float min_label_px, bool timecode, float fps) {
  const int fps_i = std::max(1, (int)std::round(fps));
  std::vector<int> cand;
  for(int v : {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000}) cand.push_back(v);
  if(timecode) { // 秒/分/時の区切りを優先して候補にする
    cand.clear();
    for(int v : {1, 2, 5, 10, 15}) cand.push_back(v);
    for(int sec : {1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600, 7200, 18000, 36000}) cand.push_back(sec * fps_i);
    std::sort(cand.begin(), cand.end());
    cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
  }
  size_t i = 0;
  while(i + 1 < cand.size() && cand[i] * px_per_frame < min_label_px) i++;
  RulerTicks t;
  t.major = cand[i];
  for(size_t j = i; j-- > 0;) {
    if(t.major % cand[j] == 0 && t.major / cand[j] >= 2) {
      t.minor = cand[j];
      break;
    }
  }
  return t;
}

// 表示テキストがpx幅に収まるようUTF-8境界で切って末尾に...を付ける。全く入らなければ空文字
static std::string ellipsize(const char* text, float width_px) {
  if(!text || width_px <= 0.0f) return "";
  if(ImGui::CalcTextSize(text).x <= width_px) return text;
  const float ell_w = ImGui::CalcTextSize("...").x;
  if(width_px <= ell_w) return "";
  std::string s(text);
  size_t end = s.size();
  while(end > 0) {
    do {
      end--;
    } while(end > 0 && (static_cast<unsigned char>(s[end]) & 0xC0) == 0x80);
    if(end == 0) return "";
    if(ImGui::CalcTextSize(s.substr(0, end).c_str()).x + ell_w <= width_px) return s.substr(0, end) + "...";
  }
  return "";
}

static std::string frame_label(int frame, bool timecode, float fps) { return timecode ? frame_to_timecode(frame, fps) : std::to_string(frame); }

inline void draw_diamond(int x, int y, float size, ImU32 color, ImDrawList* dl, bool fill_ = true) {
  const auto r = 0.607f * size / 2.0f;

  const auto c = ImVec2(x, y);
  if(fill_) {
    dl->PathLineTo(c + ImVec2(0, -r));
    dl->PathLineTo(c + ImVec2(r, 0));
    dl->PathLineTo(c + ImVec2(0, r));
    dl->PathLineTo(c + ImVec2(-r, 0));
    dl->PathFillConvex(color);
  } else {
    dl->PathLineTo(c + ImVec2(0, -r));
    dl->PathLineTo(c + ImVec2(r, 0));
    dl->PathLineTo(c + ImVec2(0, r));
    dl->PathLineTo(c + ImVec2(-r, 0));
    dl->PathStroke(color, ImDrawFlags_Closed);
  }
}

bool BeginTimeline(const char* name, FrameT* frame, FrameT* start, FrameT* end, bool* playing, float fps, const ImVec2& size) {
  if(playing != nullptr && ctx_.toggle_play) *playing = !*playing;
  ctx_.toggle_play = false;

  // フレーム冒頭のリセット
  ctx_.frame_ptr        = frame;
  ctx_.start_ptr        = start;
  ctx_.end_ptr          = end;
  ctx_.any_clip_hovered = false;
  ctx_.last_entt_hov    = nullptr;
  ctx_.snap_line        = INT_MIN;
  ctx_.clips.clear();
  ctx_.selected.clear();
  for(const auto& e : get_selected_entts())
    if(e) ctx_.selected.push_back(e.get());

  // available max height
  {
    auto height  = ImGui::GetContentRegionAvail().y;
    auto window  = ImGui::GetCurrentWindow();
    auto width_  = std::max(size.x, window->InnerClipRect.GetWidth());
    ctx_.height  = std::max<int>(8, (int)std::round(ImGui::GetTextLineHeightWithSpacing() * ctx_.row_scale));
    ctx_.fps     = fps > 0.0f ? fps : 30.0f;
    auto height_ = std::max<int>(height, ctx_.hidx * ctx_.height);
    auto pos     = ImGui::GetCursorScreenPos();

    ctx_.all_area = cutil::Rect(pos.x, pos.x + std::max<float>(1.0f, width_ - ctx_.right_strip_w), pos.y, pos.y + height_);
  }

  auto all = ctx_.all_area;

  bool open;
  const float item_height = ctx_.height;

  // scroll window
  {
    open = ImGui::BeginChild(name, ImVec2(all.w(), all.h()), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::Dummy(ImVec2(0.0f, item_height * ctx_.hidx));
  }

  auto dl = ImGui::GetWindowDrawList();

  auto bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
  dl->AddRectFilled(ImVec2(all.left() + ctx_.trackname_width, all.top()), ImVec2(all.right(), all.bottom()), IM_COL32(bg.x * 255, bg.y * 255, bg.z * 255, bg.w * 255));

  {
    auto area  = all;
    area.x.max = area.x.min + ctx_.trackname_width;
    auto col   = IM_COL32(0, 0, 0, 100);
    dl->AddRectFilled(ImVec2(area.left(), area.top() + ctx_.header_h), ImVec2(area.right(), area.bottom()), col);
  }

  dl->AddRect(ImVec2(all.left(), all.top()), ImVec2(all.right(), all.bottom()), col_.border);

  if(ctx_.first) {
    ctx_.vis_start = *start - 20;
    ctx_.vis_end   = *end + 20;
    ctx_.first     = false;
  }

  // draw header
  {
    auto he = ctx_.header_area();
    dl->AddRectFilled(ImVec2(he.x.min, he.y.min), ImVec2(he.x.max, he.y.max), col_.header_bg);
    dl->AddRect(ImVec2(he.x.min, he.y.min), ImVec2(he.x.max, he.y.max), col_.border);

    const int ruler_px = std::max<int>(1, ctx_.tl_w());
    const float ppf    = (ctx_.vis_end > ctx_.vis_start) ? (float)ruler_px / (float)(ctx_.vis_end - ctx_.vis_start) : 1.0f;
    const auto ticks   = choose_ruler_ticks(ppf, ctx_.ruler_timecode ? 96.0f : 56.0f, ctx_.ruler_timecode, ctx_.fps);
    const auto inside_ = ctx_.tl_area();
    // プレイヘッドのバッジ矩形(この範囲に重なる目盛りラベルは間引く)
    const std::string badge_txt = frame_label((int)*frame, ctx_.ruler_timecode, ctx_.fps);
    const float badge_w         = ImGui::CalcTextSize(badge_txt.c_str()).x + 10.0f;
    const float badge_cx        = std::clamp<float>((float)ctx_.f2view(*frame), inside_.x.min + badge_w / 2, inside_.x.max - badge_w / 2);
    const float badge_l = badge_cx - badge_w / 2, badge_r = badge_cx + badge_w / 2;

    dl->PushClipRect(ImVec2(inside_.x.min, all.top()), ImVec2(inside_.x.max, all.bottom()), true);
    // 副目盛り→グリッド線(縦)→主目盛り+ラベルの順に描画
    const float y0 = all.y.min + ctx_.header_h, y1 = all.bottom();
    if(ticks.minor > 0 && ticks.minor * ppf >= 5.0f) {
      for(FrameT i = (ctx_.vis_start / ticks.minor) * ticks.minor; i < ctx_.vis_end; i += ticks.minor) {
        auto x = ctx_.f2view(i);
        dl->AddLine(ImVec2(x, all.y.min + ctx_.header_h - 6), ImVec2(x, all.y.min + ctx_.header_h), IM_COL32(255, 255, 255, 70));
        dl->AddLine(ImVec2(x, y0), ImVec2(x, y1), IM_COL32(255, 255, 255, 12)); // 副グリッド
      }
    }
    for(FrameT i = (ctx_.vis_start / ticks.major) * ticks.major; i < ctx_.vis_end; i += ticks.major) {
      auto x = ctx_.f2view(i);
      dl->AddLine(ImVec2(x, all.y.min + ctx_.header_h - 12), ImVec2(x, all.y.min + ctx_.header_h), IM_COL32(255, 255, 255, 140));
      dl->AddLine(ImVec2(x, y0), ImVec2(x, y1), IM_COL32(255, 255, 255, 30)); // 主グリッド
      const std::string lbl = frame_label(i, ctx_.ruler_timecode, ctx_.fps);
      const float lw        = ImGui::CalcTextSize(lbl.c_str()).x;
      if(x + 3 + lw < badge_l - 2 || x + 3 > badge_r + 2) dl->AddText(ImVec2(x + 3, all.y.min + 1), IM_COL32(255, 255, 255, 150), lbl.c_str());
    }
    dl->PopClipRect();

    // Compositionの範囲を描画
    int st = ctx_.f2view(*start);
    int ed = ctx_.f2view(*end);
    st     = std::clamp<int>(st, he.x.min, he.x.max);
    ed     = std::clamp<int>(ed, he.x.min, he.x.max);
    dl->AddRectFilled(ImVec2(st, he.y.max - 8), ImVec2(ed, he.y.max), IM_COL32(0, 150, 255, 100));

    // レンダリング済み(キャッシュ済み)フレームをAfterEffects風に緑線で表示
    if(ctx_.active_comp) {
      for(int x = he.x.min; x < he.x.max; x++) {
        if(ctx_.active_comp->cache.is_cached(ctx_.view2f(x))) dl->AddLine(ImVec2(x, he.y.max - 10), ImVec2(x, he.y.max - 8), IM_COL32(0, 200, 0, 200));
      }
    }

    // Compositionのスタートゴールを描画し、<kbd>[</kbd>と<kbd>]</kbd>キーで終端を設定
    ImRect comp_start_ = ImRect(ImVec2(st - 2, he.y.min), ImVec2(st + 2, he.y.max));
    ImRect comp_end_   = ImRect(ImVec2(ed - 2, he.y.min), ImVec2(ed + 2, he.y.max));

    bool start_hovered = ImGui::IsMouseHoveringRect(comp_start_.Min, comp_start_.Max);
    bool end_hovered   = ImGui::IsMouseHoveringRect(comp_end_.Min, comp_end_.Max);
    // 端ハンドルのドラッグでコンポの開始/終了フレームを変更する
    if(!is_exporting() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      if(start_hovered)
        ctx_.comp_drag = 1;
      else if(end_hovered)
        ctx_.comp_drag = 2;
    }
    if(ctx_.comp_drag != 0) {
      if(ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        int f = ctx_.view2f(ImGui::GetMousePos().x);
        if(ctx_.comp_drag == 1)
          *start = std::min(f, *end - 1);
        else
          *end = std::max(f, *start + 1);
        start_hovered = ctx_.comp_drag == 1;
        end_hovered   = ctx_.comp_drag == 2;
        ImGui::SetTooltip("%s=%d", ctx_.comp_drag == 1 ? "スタート" : "エンド", ctx_.comp_drag == 1 ? *start : *end);
      } else {
        ctx_.comp_drag = 0;
      }
    } else if(start_hovered) {
      ImGui::SetTooltip("スタートフレーム=%d\nドラッグでコンポの開始位置を変更", *start);
    } else if(end_hovered) {
      ImGui::SetTooltip("エンドフレーム=%d\nドラッグでコンポの終了位置を変更", *end);
    } else if(ImGui::IsMouseHoveringRect(ImVec2(st, he.y.max - 8), ImVec2(ed, he.y.max))) {
      ImGui::SetTooltip("コンポジションの範囲 %d - %d (%d f)\nプレビュー/書き出しの対象範囲。緑の線はレンダリング済みフレーム", *start, *end, *end - *start);
    }
    if(start_hovered || end_hovered) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    dl->AddRectFilled(comp_start_.Min, comp_start_.Max, IM_COL32(0, 180, 255, start_hovered ? 255 : 200));
    dl->AddRectFilled(comp_end_.Min, comp_end_.Max, IM_COL32(0, 180, 255, end_hovered ? 255 : 200));

    // Composition範囲外を暗くする(AE参考)
    auto trk_area = ctx_.tl_area();
    trk_area.y    = all.y;
    if(st > trk_area.x.min) dl->AddRectFilled(ImVec2(trk_area.x.min, trk_area.y.min), ImVec2(st, trk_area.y.max), IM_COL32(0, 0, 0, 80));
    if(ed < trk_area.x.max) dl->AddRectFilled(ImVec2(ed, trk_area.y.min), ImVec2(trk_area.x.max, trk_area.y.max), IM_COL32(0, 0, 0, 80));
  }

  // ヘッダー左端(トラック名カラム幅ぶん): フィット/検索/タイムコードをまとめる
  {
    ImVec2 h_min(all.x.min, all.y.min);
    ImVec2 h_max(all.x.min + ctx_.trackname_width, all.y.min + ctx_.header_h);
    dl->AddRectFilled(h_min, h_max, col_.header_bg);
    dl->AddRect(h_min, h_max, col_.border);

    float item_h  = ctx_.header_h;
    float bx      = h_min.x + 2;
    auto icon_btn = [&](const char* icon, bool highlighted) -> bool {
      ImVec2 p0(bx, h_min.y);
      ImVec2 p1(bx + item_h, h_max.y);
      bool hov = ImGui::IsMouseHoveringRect(p0, p1);
      if(hov) dl->AddRectFilled(p0, p1, IM_COL32(255, 255, 255, 30));
      dl->AddText(ImVec2(p0.x + 2, p0.y + 2), highlighted ? IM_COL32(120, 180, 255, 255) : IM_COL32(255, 255, 255, 180), icon);
      bx += item_h;
      return hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    };

    if(icon_btn(ICON_FA_EXPAND, false)) ctx_.pending_fit = true;
    if(icon_btn(ICON_FA_MAGNIFYING_GLASS, ctx_.search_open)) ctx_.search_open = !ctx_.search_open;

    float rest_w = h_max.x - bx - 4;
    if(ctx_.search_open) {
      ImGui::SetCursorScreenPos(ImVec2(bx, h_min.y + 1));
      ImGui::PushID("tl_search");
      ImGui::SetNextItemWidth(std::max(10.0f, rest_w));
      if(ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere(-1);
      ImGui::InputText("##layer_search", ctx_.layer_search_buf, sizeof(ctx_.layer_search_buf));
      ImGui::PopID();
    } else {
      std::string tc = frame_to_timecode((int)*frame, fps);
      dl->AddText(ImVec2(bx + 2, h_min.y + 2), IM_COL32(255, 255, 255, 200), tc.c_str());
    }
  }


  // *frameは呼び出し元(timeline_window.cpp)がComposition::frame(atomic)をスナップショットしたローカル変数
  {
    auto x = ctx_.f2view(*frame);

    const auto inside_          = ctx_.tl_area();
    const std::string badge_txt = frame_label((int)*frame, ctx_.ruler_timecode, ctx_.fps);
    const float badge_w         = ImGui::CalcTextSize(badge_txt.c_str()).x + 10.0f;
    const float badge_cx        = std::clamp<float>((float)x, inside_.x.min + badge_w / 2, inside_.x.max - badge_w / 2);
    ImVec2 p1(badge_cx - badge_w / 2, ctx_.all_area.y.min + 1);
    ImVec2 p2(badge_cx + badge_w / 2, ctx_.all_area.y.min + ctx_.header_h - 1);
    const bool badge_hov = ImGui::IsMouseHoveringRect(ImVec2(p1.x - 4, p1.y), ImVec2(p2.x + 4, p2.y));
    auto col             = badge_hov ? IM_COL32(255, 60, 60, 255) : IM_COL32(200, 30, 30, 235);
    // トラック名カラム(サイドバー)へのはみ出しを防ぐ
    dl->PushClipRect(ImVec2(inside_.x.min, all.top()), ImVec2(all.right(), all.bottom()), true);
    dl->AddLine(ImVec2(x, ctx_.all_area.y.min + ctx_.header_h), ImVec2(x, ctx_.all_area.bottom()), col);
    dl->AddRectFilled(p1, p2, col, 3.0f);
    dl->AddText(ImVec2(p1.x + 5, p1.y + (p2.y - p1.y - ImGui::GetTextLineHeight()) / 2), col_.cursor_label, badge_txt.c_str());
    dl->AddTriangleFilled(ImVec2(x - 4, p2.y), ImVec2(x + 4, p2.y), ImVec2(x, p2.y + 4), col);
    dl->PopClipRect();

    auto h_             = ctx_.header_area();
    bool in_header_area = ImGui::IsMouseHoveringRect(ImVec2(std::max<float>(h_.x.min, inside_.x.min), h_.y.min), ImVec2(h_.x.max, h_.y.max));
    bool lclick         = ImGui::IsMouseClicked(ImGuiMouseButton_Left) || ImGui::IsMouseDragging(ImGuiMouseButton_Left);
    if(in_header_area && lclick && ctx_.comp_drag == 0 && !ImGui::IsPopupOpen("tl_ruler_ctx")) {
      *frame = ctx_.view2f(ImGui::GetMousePos().x);
    }
  }

  ctx_.hidx      = 0;
  ctx_.cur_frame = *frame;
  // vis_start/vis_endはズーム/パン状態のため*start/*end(Composition境界)で上書きしない(上書きするとズームが毎フレーム巻き戻る)
  return open;
}

// 選択Entityをクリップ操作(分割/複製/削除/有効切替)の対象にする
static std::vector<Ref<Entity>> selected_refs() { return get_selected_entts(); }

static void set_selected_cache() {
  ctx_.selected.clear();
  for(const auto& e : get_selected_entts())
    if(e) ctx_.selected.push_back(e.get());
}

static bool is_selected_entt(const Entity* e) { return std::find(ctx_.selected.begin(), ctx_.selected.end(), e) != ctx_.selected.end(); }

static const char* entt_type_label(const Entity* e) {
  switch(e->getType()) {
    case EntityType_Movie: return "動画";
    case EntityType_Audio: return "音声";
    case EntityType_Image: return "画像";
    case EntityType_3DText: return "テキスト";
    case EntityType_Polygon: return "図形";
    case EntityType_Framebuffer: return "フレームバッファ";
    case EntityType_Group: return "グループ制御";
    case EntityType_Scene: return "コンポ参照";
    case EntityType_SceneAudio: return "コンポ音声参照";
    case EntityType_Custom: return "カスタムオブジェクト";
    case EntityType_Midi: return "MIDI";
    case EntityType_Camera: return "カメラ制御";
    case EntityType_SceneChange: return "シーンチェンジ";
    default: return "オブジェクト";
  }
}

// 「ここに追加」メニュー本体。項目が選ばれたら遅延追加(EndTimeline)を予約してtrueを返す
bool TimelineAddEntityMenu(int frame, int layer) {
  auto add = [&](EntityType type, int shape = -1) {
    ctx_.pending_add = {true, type, shape, std::max(0, frame), layer};
    return true;
  };
  bool picked = false;
  if(ImGui::MenuItem(ICON_FA_FONT " テキスト")) picked = add(EntityType_3DText);
  if(ImGui::MenuItem(ICON_FA_IMAGE " 画像")) picked = add(EntityType_Image);
  if(ImGui::MenuItem(ICON_FA_VIDEO " 動画")) picked = add(EntityType_Movie);
  if(ImGui::MenuItem(ICON_FA_MUSIC " 音声")) picked = add(EntityType_Audio);
#ifdef MOVUTL_DAW
  if(ImGui::MenuItem(ICON_FA_KEYBOARD " MIDI")) picked = add(EntityType_Midi);
#endif
  if(ImGui::BeginMenu(ICON_FA_DRAW_POLYGON " 図形")) {
    static const char* names[]     = {"三角形", "四角形", "六角形", "円"};
    static const ShapeType types[] = {ShapeType_Triangle, ShapeType_Rect, ShapeType_Hexagon, ShapeType_Circle};
    for(int i = 0; i < 4; i++)
      if(ImGui::MenuItem(names[i])) picked = add(EntityType_Polygon, (int)types[i]);
    ImGui::EndMenu();
  }
  if(ImGui::MenuItem(ICON_FA_TV " フレームバッファ")) picked = add(EntityType_Framebuffer);
  if(ImGui::MenuItem(ICON_FA_LAYER_GROUP " グループ制御")) picked = add(EntityType_Group);
  if(ImGui::MenuItem(ICON_FA_VIDEO " カメラ制御")) picked = add(EntityType_Camera);
  if(ImGui::MenuItem(ICON_FA_SHUFFLE " シーンチェンジ")) picked = add(EntityType_SceneChange);
  if(ImGui::MenuItem(ICON_FA_GLOBE " コンポ参照")) picked = add(EntityType_Scene);
  return picked;
}

bool* TimelineSnapFlag() { return &ctx_.snap; }

bool GetTimelineViewRange(FrameT* start, FrameT* end) {
  if(start) *start = ctx_.vis_start;
  if(end) *end = ctx_.vis_end;
  return ctx_.vis_end > ctx_.vis_start;
}

// 表示幅(フレーム数)を、表示中心を保ったまま変更する
void SetTimelineVisibleFrames(float frames) {
  frames         = std::max(10.0f, frames);
  float center   = (ctx_.vis_start + ctx_.vis_end) / 2.0f;
  ctx_.vis_start = (int)(center - frames / 2);
  ctx_.vis_end   = (int)(center + frames / 2);
}

// 破壊的操作(クリップ/レイヤーの追加・削除)。ループ中のvector破壊を避けるためEndTimelineで適用する
static void apply_pending_ops(Composition* cp) {
  if(!cp) return;
  bool changed = false;

  if(ctx_.pending_add.valid) {
    auto pa          = ctx_.pending_add;
    ctx_.pending_add = {};
    // 既存のadd_new_*は空きレイヤーへ追加するため、追加後に増えたEntityを指定レイヤーへ移す
    std::vector<Ref<Entity>> before;
    {
      std::lock_guard<std::mutex> lock(cp->mtx);
      for(auto& l : cp->layers)
        for(auto& e : l.entts) before.push_back(e);
    }
    if(pa.shape >= 0)
      add_new_shape_track("shape", pa.frame, pa.frame + 100, (ShapeType)pa.shape);
    else
      add_new_track(pa.type == EntityType_3DText ? "text" : "obj", pa.type, pa.frame, pa.frame + 100);
    if(pa.layer >= 0 && pa.layer < (int)cp->layers.size()) {
      std::lock_guard<std::mutex> lock(cp->mtx);
      for(auto& l : cp->layers) {
        for(size_t i = 0; i < l.entts.size();) {
          auto e = l.entts[i];
          if(e && std::find(before.begin(), before.end(), e) == before.end() && &l != &cp->layers[pa.layer]) {
            l.entts.erase(l.entts.begin() + i);
            cp->layers[pa.layer].entts.push_back(e);
          } else {
            i++;
          }
        }
      }
    }
    changed = true;
  }

  if(ctx_.pending_clip_op != 0) {
    const int op         = ctx_.pending_clip_op;
    ctx_.pending_clip_op = 0;
    if(op == 1) { // 分割: 右クリックした位置で分割し、プレイヘッドは動かさない
      int prev = cp->frame.load();
      cp->frame.store(ctx_.ctx_frame);
      run_command("split");
      cp->frame.store(prev);
    } else if(op == 2) { // 複製: 元クリップの直後の同じレイヤーへ
      for(const auto& src : selected_refs()) {
        auto clone = duplicate_asset(src);
        if(!clone) continue;
        int len        = src->fend_ - src->fstart_;
        clone->fstart_ = src->fend_;
        clone->fend_   = src->fend_ + len;
        int layer      = -1;
        for(int li = 0; li < (int)cp->layers.size() && layer < 0; li++)
          for(const auto& e : cp->layers[li].entts)
            if(e == src) layer = li;
        cp->insert_entity(clone, layer);
      }
    } else if(op == 3) { // 削除
      auto sel = selected_refs();
      {
        std::lock_guard<std::mutex> lock(cp->mtx);
        for(auto& l : cp->layers)
          for(const auto& d : sel) l.entts.erase(std::remove(l.entts.begin(), l.entts.end(), d), l.entts.end());
      }
      clear_selected_entts();
    } else if(op == 4) { // 有効/無効(選択の一つでも有効なら全て無効、全て無効なら全て有効)
      auto sel        = selected_refs();
      bool any_active = false;
      for(const auto& e : sel) any_active |= e->active_;
      for(const auto& e : sel) e->active_ = !any_active;
    } else if(op == 5) { // 上のオブジェクトでクリッピング(選択の一つでもONなら全てOFF、全てOFFなら全てON)
      auto sel     = selected_refs();
      bool any_clp = false;
      for(const auto& e : sel) any_clp |= e->clipping_up_;
      for(const auto& e : sel) e->clipping_up_ = !any_clp;
    } else if(op == 6) { // カメラ制御の対象(同上)
      auto sel     = selected_refs();
      bool any_cam = false;
      for(const auto& e : sel) any_cam |= e->camera_ctrl_;
      for(const auto& e : sel) e->camera_ctrl_ = !any_cam;
    }
    changed = true;
  }

  {
    std::lock_guard<std::mutex> lock(cp->mtx);
    if(ctx_.pending_insert_layer >= 0) {
      int at = std::clamp(ctx_.pending_insert_layer, 0, (int)cp->layers.size());
      cp->layers.insert(cp->layers.begin() + at, TrackLayer());
      ctx_.pending_insert_layer = -1;
      ctx_.solo_layer           = -1; // レイヤーindexがずれるためソロ状態は解除
      changed                   = true;
    }
    if(ctx_.pending_delete_layer >= 0 && ctx_.pending_delete_layer < (int)cp->layers.size()) {
      cp->layers.erase(cp->layers.begin() + ctx_.pending_delete_layer);
      ctx_.pending_delete_layer = -1;
      ctx_.solo_layer           = -1;
      changed                   = true;
    }
    if(ctx_.pending_entt_move) {
      int to = ctx_.pending_entt_layer;
      if(to >= 0 && to < (int)cp->layers.size()) {
        for(auto& l : cp->layers) {
          auto it = std::find_if(l.entts.begin(), l.entts.end(), [&](const Ref<Entity>& e) { return e.get() == ctx_.pending_entt_move; });
          if(it == l.entts.end()) continue;
          auto ref = *it;
          l.entts.erase(it);
          cp->layers[to].entts.push_back(ref);
          changed = true;
          break;
        }
      }
      ctx_.pending_entt_move = nullptr;
    }
    if(ctx_.pending_move_layer >= 0 && ctx_.pending_move_layer < (int)cp->layers.size()) {
      int i = ctx_.pending_move_layer;
      int j = i + ctx_.pending_move_dir;
      if(j >= 0 && j < (int)cp->layers.size()) std::swap(cp->layers[i], cp->layers[j]);
      ctx_.pending_move_layer = -1;
      ctx_.solo_layer         = -1;
      changed                 = true;
    }
  }
  if(changed) cp->invalidate_cache_all();
}

int EndTimeline() {
  int return_value       = ctx_.cur_frame;
  const bool hovered_win = ImGui::IsWindowHovered();
  const auto inside      = ctx_.tl_area();
  const auto all         = ctx_.all_area;
  const ImVec2 mouse     = ImGui::GetMousePos();
  auto dl                = ImGui::GetWindowDrawList();

  if(hovered_win) {
    if(ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
      auto delta = ImGui::GetIO().MouseDelta.x;
      ctx_.vis_start -= delta;
      ctx_.vis_end -= delta;
    }

    // マウスホイール: Shift押下時は横スクロール(パン)、それ以外(Ctrl含む)はカーソル位置中心のズーム
    auto delta = ImGui::GetIO().MouseWheel;
    if(delta != 0.0f) {
      if(ImGui::GetIO().KeyShift) {
        auto pan_amount = (ctx_.vis_end - ctx_.vis_start) * (-delta) / 20.0f;
        ctx_.vis_start += pan_amount;
        ctx_.vis_end += pan_amount;
      } else {
        auto center    = ctx_.view2f(mouse.x);
        auto scale     = 1.0f + delta / 15.0f;
        ctx_.vis_start = center + (ctx_.vis_start - center) * scale;
        ctx_.vis_end   = center + (ctx_.vis_end - center) * scale;
      }
    }
    return_value = ctx_.cur_frame;
  }

  const ImRect rows(ImVec2(inside.left(), all.y.min + ctx_.header_h), ImVec2(inside.right(), all.bottom()));
  const bool in_rows    = rows.Contains(mouse);
  const bool in_ruler   = ImRect(ImVec2(inside.left(), all.y.min), ImVec2(inside.right(), all.y.min + ctx_.header_h)).Contains(mouse);
  const bool can_edit   = !is_exporting();
  const bool popup_open = ImGui::IsPopupOpen("tl_empty_ctx") || ImGui::IsPopupOpen("tl_ruler_ctx") || ImGui::IsPopupOpen("tl_clip_ctx");

  // ---- ラバーバンド範囲選択(空きトラックのドラッグ) ----
  if(can_edit && !ctx_.rb_active && !popup_open && hovered_win && in_rows && !ctx_.any_clip_hovered && ctx_.dragging_entt == nullptr && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    ctx_.rb_active = true;
    ctx_.rb_moved  = false;
    ctx_.rb_start  = mouse;
  }
  if(ctx_.rb_active) {
    ImRect r(ImVec2(std::min(mouse.x, ctx_.rb_start.x), std::min(mouse.y, ctx_.rb_start.y)), ImVec2(std::max(mouse.x, ctx_.rb_start.x), std::max(mouse.y, ctx_.rb_start.y)));
    if(ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      if(std::abs(mouse.x - ctx_.rb_start.x) + std::abs(mouse.y - ctx_.rb_start.y) > 4.0f) ctx_.rb_moved = true;
      if(ctx_.rb_moved) {
        dl->PushClipRect(rows.Min, rows.Max, true);
        dl->AddRectFilled(r.Min, r.Max, IM_COL32(80, 150, 255, 40));
        dl->AddRect(r.Min, r.Max, IM_COL32(120, 180, 255, 200));
        dl->PopClipRect();
      }
    } else {
      const bool additive = ImGui::GetIO().KeyCtrl || ImGui::GetIO().KeyShift;
      if(!ctx_.rb_moved) {
        if(!additive) clear_selected_entts(); // 何もない所のクリック=選択解除
      } else {
        std::vector<Ref<Entity>> picked = additive ? get_selected_entts() : std::vector<Ref<Entity>>();
        for(const auto& c : ctx_.clips)
          if(c.r.Overlaps(r) && std::find(picked.begin(), picked.end(), c.e) == picked.end()) picked.push_back(c.e);
        select_entts(picked);
      }
      ctx_.rb_active = false;
      set_selected_cache();
    }
  }

  // ---- 右クリック: 空トラック / 定規 ----
  if(can_edit && hovered_win && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
    if(in_rows && !ctx_.any_clip_hovered) {
      int li           = (int)((mouse.y - rows.Min.y) / std::max(1, ctx_.height));
      bool valid_layer = ctx_.active_comp && li >= 0 && li < (int)ctx_.active_comp->layers.size();
      ctx_.ctx_layer   = valid_layer ? li : -1;
      ctx_.ctx_frame   = ctx_.view2f((int)mouse.x);
      ImGui::OpenPopup("tl_empty_ctx");
    } else if(in_ruler) {
      ctx_.ctx_frame = ctx_.view2f((int)mouse.x);
      ImGui::OpenPopup("tl_ruler_ctx");
    }
  }
  if(ImGui::BeginPopup("tl_empty_ctx")) {
    if(ImGui::BeginMenu(ICON_FA_PLUS " ここに追加")) {
      TimelineAddEntityMenu(ctx_.ctx_frame, ctx_.ctx_layer);
      ImGui::EndMenu();
    }
    if(ImGui::MenuItem(ICON_FA_LOCATION_ARROW " ここに再生ヘッドを移動") && ctx_.frame_ptr) *ctx_.frame_ptr = ctx_.ctx_frame;
    if(ImGui::MenuItem(ICON_FA_EXPAND " 全体をフィット")) ctx_.pending_fit = true;
    ImGui::EndPopup();
  }
  if(ImGui::BeginPopup("tl_ruler_ctx")) {
    if(ImGui::MenuItem(ICON_FA_LOCATION_ARROW " ここに再生ヘッドを移動") && ctx_.frame_ptr) *ctx_.frame_ptr = ctx_.ctx_frame;
    if(ImGui::MenuItem(ICON_FA_EXPAND " コンポ範囲にフィット")) ctx_.pending_fit = true;
    if(ImGui::MenuItem("コンポの開始をここにする") && ctx_.start_ptr && ctx_.end_ptr) *ctx_.start_ptr = std::min(ctx_.ctx_frame, *ctx_.end_ptr - 1);
    if(ImGui::MenuItem("コンポの終了をここにする") && ctx_.start_ptr && ctx_.end_ptr) *ctx_.end_ptr = std::max(ctx_.ctx_frame, *ctx_.start_ptr + 1);
    ImGui::Separator();
    if(ImGui::MenuItem("表記: フレーム", nullptr, !ctx_.ruler_timecode)) ctx_.ruler_timecode = false;
    if(ImGui::MenuItem("表記: タイムコード", nullptr, ctx_.ruler_timecode)) ctx_.ruler_timecode = true;
    ImGui::EndPopup();
  }

  // ---- クリップの右クリックメニュー(BeginTrackが開く) ----
  if(ImGui::BeginPopup("tl_clip_ctx")) {
    const bool multi = ctx_.selected.size() > 1;
    if(ImGui::MenuItem(ICON_FA_SCISSORS " ここで分割")) ctx_.pending_clip_op = 1;
    if(ImGui::MenuItem(multi ? ICON_FA_COPY " 選択を複製" : ICON_FA_COPY " 複製")) ctx_.pending_clip_op = 2;
    bool now_active = ctx_.ctx_entt ? ctx_.ctx_entt->active_ : true;
    if(ImGui::MenuItem(now_active ? ICON_FA_EYE_SLASH " 無効にする" : ICON_FA_EYE " 有効にする")) ctx_.pending_clip_op = 4;
    bool now_clip = ctx_.ctx_entt ? ctx_.ctx_entt->clipping_up_ : false;
    if(ImGui::MenuItem(ICON_FA_CROP_SIMPLE " 上のオブジェクトでクリッピング", nullptr, now_clip)) ctx_.pending_clip_op = 5;
    bool now_cam = ctx_.ctx_entt ? ctx_.ctx_entt->camera_ctrl_ : false;
    if(ImGui::MenuItem(ICON_FA_VIDEO " カメラ制御の対象", nullptr, now_cam)) ctx_.pending_clip_op = 6;
    ImGui::Separator();
    if(ImGui::MenuItem(multi ? ICON_FA_TRASH " 選択を削除" : ICON_FA_TRASH " 削除")) ctx_.pending_clip_op = 3;
    ImGui::EndPopup();
  }

  // ---- 列幅リサイズ(トラック名カラムとトラックの境界をドラッグ) ----
  {
    ImRect grip(ImVec2(inside.left() - 3, all.y.min), ImVec2(inside.left() + 3, all.bottom()));
    bool over = ImGui::IsMouseHoveringRect(grip.Min, grip.Max);
    if(can_edit && over && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) ctx_.name_col_drag = true;
    if(ctx_.name_col_drag) {
      if(ImGui::IsMouseDown(ImGuiMouseButton_Left))
        ctx_.trackname_width = std::clamp<int>((int)(mouse.x - all.left()), 96, 420);
      else
        ctx_.name_col_drag = false;
    }
    if(over || ctx_.name_col_drag) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
  }

  // ---- 遅延操作 ----
  apply_pending_ops(ctx_.active_comp ? ctx_.active_comp : Composition::GetActiveComp());

  if(ctx_.cur_frame && ctx_.vis_start && ctx_.vis_end) {
    ctx_.cur_frame = std::clamp<int>(ctx_.cur_frame, ctx_.vis_start, ctx_.vis_end);
  }

  // ---- スナップのガイド線 ----
  if(ctx_.snap_line != INT_MIN) {
    int x = ctx_.f2view(ctx_.snap_line);
    dl->PushClipRect(ImVec2(inside.left(), all.top()), ImVec2(inside.right(), all.bottom()), true);
    dl->AddLine(ImVec2(x, all.y.min + ctx_.header_h), ImVec2(x, all.bottom()), IM_COL32(255, 220, 60, 220), 1.5f);
    dl->PopClipRect();
  }

  // ---- レイヤー行の区切り線 ----
  auto line_col = ImGui::GetStyle().Colors[ImGuiCol_Border];
  ImVec2 p1(all.left(), all.y.min + ctx_.header_h);
  ImVec2 p2(all.right(), all.y.min + ctx_.header_h);
  for(int i = 0; i < ctx_.hidx + 1; i++) {
    dl->AddLine(p1, p2, IM_COL32(line_col.x * 255, line_col.y * 255, line_col.z * 255, line_col.w * 255 * 0.8f), 1.0f);
    p1.y += ctx_.height;
    p2.y += ctx_.height;
  }

  ImGui::EndChild();
  return return_value;
}

// ソロ: 押した行以外の全レイヤーをinactiveにする。もう一度押すと押す前のactiveへ戻す(押す前の状態はUI側のctx_に保持)
static void toggle_solo(Composition* cp, int idx) {
  {
    std::lock_guard<std::mutex> lock(cp->mtx);
    if(ctx_.solo_layer == idx) {
      for(size_t i = 0; i < cp->layers.size() && i < ctx_.solo_saved.size(); i++) cp->layers[i].active = ctx_.solo_saved[i];
      ctx_.solo_layer = -1;
    } else {
      if(ctx_.solo_layer < 0) {
        ctx_.solo_saved.clear();
        for(auto& l : cp->layers) ctx_.solo_saved.push_back(l.active);
      }
      for(size_t i = 0; i < cp->layers.size(); i++) cp->layers[i].active = ((int)i == idx);
      ctx_.solo_layer = idx;
    }
  }
  cp->invalidate_cache_all();
}

bool BeginLayer(Composition* cp, int layer_idx) {
  MU_ASSERT(cp);
  MU_ASSERT(layer_idx >= 0 && layer_idx < (int)cp->layers.size());
  TrackLayer* layer     = &cp->layers[layer_idx];
  ctx_.active_comp      = cp;
  ctx_.cur_layer_active = layer->active;

  auto dl     = ImGui::GetWindowDrawList();
  auto inside = ctx_.tl_area();

  int x    = ctx_.all_area.left() + ImGui::GetStyle().ItemSpacing.x;
  int htop = ctx_.layer_y1();
  int hbtm = ctx_.layer_y2();

  int btn_w  = hbtm - htop; // 目/ソロボタン用の正方形幅(行高さに合わせる)
  int name_x = x + btn_w * 2 + 4;

  ImRect sidebar(ImVec2(x, htop), ImVec2(inside.left(), hbtm));
  ImRect eye_rect(ImVec2(x, htop), ImVec2(x + btn_w, hbtm));
  ImRect solo_rect(ImVec2(x + btn_w, htop), ImVec2(x + btn_w * 2, hbtm));
  bool eye_hovered     = ImGui::IsMouseHoveringRect(eye_rect.Min, eye_rect.Max);
  bool solo_hovered    = ImGui::IsMouseHoveringRect(solo_rect.Min, solo_rect.Max);
  bool sidebar_hovered = ImGui::IsMouseHoveringRect(sidebar.Min, sidebar.Max) && !eye_hovered && !solo_hovered;

  const bool solo_on = ctx_.solo_layer == layer_idx;
  if(!is_exporting() && eye_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    layer->active   = !layer->active;
    ctx_.solo_layer = -1; // 手動で切り替えたらソロ状態は解除(復元しない)
    cp->invalidate_cache_all();
  }
  if(!is_exporting() && solo_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) toggle_solo(cp, layer_idx);
  if(eye_hovered) ImGui::SetTooltip(layer->active ? "レイヤーを非表示(ミュート)にする" : "レイヤーを表示する");
  if(solo_hovered) ImGui::SetTooltip(solo_on ? "ソロを解除(元の表示状態へ戻す)" : "ソロ: このレイヤー以外を非表示にする");

  bool editing = ctx_.editing_layer_idx == layer_idx;
  if(editing) {
    ImGui::SetCursorScreenPos(ImVec2(name_x, htop));
    ImGui::PushID(layer_idx);
    ImGui::SetNextItemWidth(inside.left() - name_x);
    bool done = ImGui::InputText("##layer_name_edit", ctx_.editing_layer_buf, sizeof(ctx_.editing_layer_buf), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
    if(ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere(-1);
    if(done || ImGui::IsItemDeactivated()) {
      layer->name            = ctx_.editing_layer_buf;
      ctx_.editing_layer_idx = -1;
    }
    ImGui::PopID();
  } else {
    if(sidebar_hovered) ImGui::SetTooltip("%s\nダブルクリックで名前を変更 / 右クリックでメニュー", layer->name.c_str());
    if(!is_exporting() && sidebar_hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
      ctx_.editing_layer_idx = layer_idx;
      std::snprintf(ctx_.editing_layer_buf, sizeof(ctx_.editing_layer_buf), "%s", layer->name.c_str());
    }
  }

  // 行背景: 交互の濃淡 / 選択中クリップのある行 / マウスホバー
  ImRect R(ImVec2(inside.left(), htop), ImVec2(inside.right(), hbtm));
  if(layer_idx % 2 == 1) dl->AddRectFilled(R.Min, R.Max, IM_COL32(255, 255, 255, 7));
  bool row_selected = false;
  for(const auto& e : layer->entts)
    if(e && is_selected_entt(e.get())) row_selected = true;
  if(row_selected) dl->AddRectFilled(R.Min, R.Max, IM_COL32(90, 140, 255, 22));
  bool line_hovered = ImGui::IsMouseHoveringRect(R.Min, R.Max);
  if(line_hovered) dl->AddRectFilled(R.Min, R.Max, IM_COL32(255, 255, 255, 14));

  // グループ制御が効くレイヤー(かつグループの表示期間)を薄い半透明でハイライトする(選択の有無は問わない)
  for(int gl = 0; gl < (int)cp->layers.size(); gl++) {
    for(const auto& g : cp->layers[gl].entts) {
      if(!g || g->getType() != EntityType_Group || !static_cast<GroupEntt*>(g.get())->affects(gl, layer_idx)) continue;
      dl->AddRectFilled(ImVec2(ctx_.f2view(g->fstart_), htop), ImVec2(ctx_.f2view(g->fend_), hbtm), IM_COL32(255, 200, 60, 18));
    }
  }

  // 見出し背景(選択行は少し明るく)とボタン
  dl->AddRectFilled(sidebar.Min, sidebar.Max, layer->active ? (row_selected ? IM_COL32(52, 56, 68, 255) : IM_COL32(40, 40, 40, 255)) : IM_COL32(20, 20, 20, 255));
  if(!layer->active) dl->AddRectFilled(ImVec2(inside.left(), htop), ImVec2(inside.right(), hbtm), IM_COL32(0, 0, 0, 110)); // 非表示レイヤーはトラック部分も暗くする
  if(eye_hovered) dl->AddRectFilled(eye_rect.Min, eye_rect.Max, IM_COL32(255, 255, 255, 25));
  const float ty_icon = htop + (btn_w - ImGui::GetTextLineHeight()) / 2;
  dl->AddText(ImVec2(x + 2, ty_icon), layer->active ? IM_COL32(255, 255, 255, 230) : IM_COL32(255, 255, 255, 90), layer->active ? ICON_FA_EYE : ICON_FA_EYE_SLASH);
  // ソロボタン
  {
    if(solo_on)
      dl->AddRectFilled(ImVec2(solo_rect.Min.x + 1, solo_rect.Min.y + 1), ImVec2(solo_rect.Max.x - 1, solo_rect.Max.y - 1), IM_COL32(230, 160, 30, 255), 3.0f);
    else if(solo_hovered)
      dl->AddRectFilled(solo_rect.Min, solo_rect.Max, IM_COL32(255, 255, 255, 25));
    auto tsz = ImGui::CalcTextSize("S");
    dl->AddText(ImVec2(solo_rect.Min.x + (btn_w - tsz.x) / 2, htop + (btn_w - tsz.y) / 2), solo_on ? IM_COL32(20, 20, 20, 255) : IM_COL32(255, 255, 255, 150), "S");
  }

  if(!editing) {
    const char* search = ctx_.layer_search_buf;
    bool dim           = !layer->active || (search[0] != '\0' && !strstr(layer->name.c_str(), search));
    std::string shown  = ellipsize(layer->name.c_str(), (float)(inside.left() - name_x - 6));
    auto tsz           = ImGui::CalcTextSize(shown.c_str());
    float ty           = htop + ((hbtm - htop) - tsz.y) / 2.0f;
    dl->AddText(ImVec2(name_x, ty), dim ? IM_COL32(255, 255, 255, 90) : IM_COL32(255, 255, 255, 235), shown.c_str());
  }

  const std::string popup_id = "layer_ctx_" + std::to_string(layer_idx);
  if(!is_exporting() && sidebar_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) ImGui::OpenPopup(popup_id.c_str());
  if(ImGui::BeginPopup(popup_id.c_str())) {
    if(ImGui::MenuItem(ICON_FA_PEN " 名前を変更")) {
      ctx_.editing_layer_idx = layer_idx;
      std::snprintf(ctx_.editing_layer_buf, sizeof(ctx_.editing_layer_buf), "%s", layer->name.c_str());
    }
    if(ImGui::MenuItem(layer->active ? ICON_FA_EYE_SLASH " 非表示にする" : ICON_FA_EYE " 表示する")) {
      layer->active   = !layer->active;
      ctx_.solo_layer = -1;
      cp->invalidate_cache_all();
    }
    if(ImGui::MenuItem("S ソロ", nullptr, solo_on)) toggle_solo(cp, layer_idx);
    ImGui::Separator();
    if(ImGui::MenuItem(ICON_FA_ARROW_UP " 上にレイヤーを挿入")) ctx_.pending_insert_layer = layer_idx;
    if(ImGui::MenuItem(ICON_FA_ARROW_DOWN " 下にレイヤーを挿入")) ctx_.pending_insert_layer = layer_idx + 1;
    if(ImGui::MenuItem("上へ移動")) {
      ctx_.pending_move_layer = layer_idx;
      ctx_.pending_move_dir   = -1;
    }
    if(ImGui::MenuItem("下へ移動")) {
      ctx_.pending_move_layer = layer_idx;
      ctx_.pending_move_dir   = 1;
    }
    if(ImGui::BeginMenu("行の高さ")) {
      if(ImGui::MenuItem("小 (S)", nullptr, ctx_.row_scale < 0.9f)) ctx_.row_scale = 0.8f;
      if(ImGui::MenuItem("中 (M)", nullptr, ctx_.row_scale >= 0.9f && ctx_.row_scale < 1.15f)) ctx_.row_scale = 1.0f;
      if(ImGui::MenuItem("大 (L)", nullptr, ctx_.row_scale >= 1.15f)) ctx_.row_scale = 1.4f;
      ImGui::EndMenu();
    }
    ImGui::Separator();
    if(ImGui::MenuItem(ICON_FA_TRASH " 削除")) ctx_.pending_delete_layer = layer_idx;
    ImGui::EndPopup();
  }
  return true;
}

void EndLayer() { ctx_.hidx++; }

bool IsTimeline_LineHovered() {
  auto dh   = (float)ctx_.height;
  auto h    = dh * (ctx_.hidx - 1);
  auto all  = ctx_.all_area;
  all.y     = all.y.shift(h);
  all.y.max = all.y.min + dh;

  ImRect rect(ImVec2(all.left(), all.top()), ImVec2(all.left() + ctx_.trackname_width, all.top() + dh));
  return ImGui::IsMouseHoveringRect(rect.Min, rect.Max);
}

bool IsTimelineKeyHovered() { return ctx_.last_entt_hov; }

bool IsTimelineClickedLeftButton() { return ctx_.last_entt_hov; }

// 色を明るさ倍率で調整する(アルファは保持)
static ImU32 scale_color(ImU32 c, float k) {
  ImVec4 v = ImGui::ColorConvertU32ToFloat4(c);
  v.x      = std::clamp(v.x * k, 0.0f, 1.0f);
  v.y      = std::clamp(v.y * k, 0.0f, 1.0f);
  v.z      = std::clamp(v.z * k, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(v);
}

// 他クリップの端/プレイヤヘッド/コンポ範囲などスナップ先候補を集める
static void collect_snap_points(Entity* self) {
  ctx_.snap_points.clear();
  ctx_.snap_points.push_back(ctx_.cur_frame);
  if(ctx_.start_ptr) ctx_.snap_points.push_back(*ctx_.start_ptr);
  if(ctx_.end_ptr) ctx_.snap_points.push_back(*ctx_.end_ptr);
  if(!ctx_.active_comp) return;
  for(const auto& e : ctx_.active_comp->get_all_entities()) {
    if(e.get() == self) continue;
    bool moving = false;
    for(auto& [o, ofs, ofe] : ctx_.drag_group_orig) moving |= (o == e.get());
    if(moving) continue;
    ctx_.snap_points.push_back(e->fstart_);
    ctx_.snap_points.push_back(e->fend_);
  }
}

// frame_candidatesの中でtargetに最も近いスナップ点(閾値内)を返す。無ければfalse
static bool nearest_snap(int target, int thr_frames, int* out) {
  int best = INT_MAX;
  for(int p : ctx_.snap_points) {
    int d = std::abs(p - target);
    if(d <= thr_frames && d < best) {
      best = d;
      *out = p;
    }
  }
  return best != INT_MAX;
}

bool BeginTrack(const Ref<Entity>& entity) {
  MU_ASSERT(entity);
  const char* name = entity->name.c_str();
  int* start       = &entity->fstart_;
  int* end         = &entity->fend_;
  int htop         = ctx_.layer_y1();

  constexpr int kEdgeW = 5; // 左右端のドラッグ判定幅(px)

  int fs = ctx_.f2view(*start);
  int fe = ctx_.f2view(*end);
  ImRect rect(ImVec2(fs, htop), ImVec2(fe, htop + ctx_.height));
  const auto inside_hit = ctx_.tl_area();
  auto mouse_x          = ImGui::GetMousePos().x;
  // レイヤー名カラムやウィンドウ外にはみ出した部分ではホバー扱いにしない
  bool hovered = ImGui::IsMouseHoveringRect(rect.Min, rect.Max) && mouse_x >= inside_hit.left() && mouse_x < inside_hit.right() && !ImGui::IsPopupOpen("tl_clip_ctx");
  if(hovered) {
    ctx_.last_entt_hov    = entity.get();
    ctx_.any_clip_hovered = true;
  }
  ctx_.clips.push_back({entity, rect});

  // 中間点(キーフレーム)の集約ダイヤのヒット判定。クリップ本体move/resizeドラッグより優先させる
  constexpr int kKfHitR = 4; // ダイヤのヒット半径(px)
  auto animated_frames  = entity->collect_animated_frames();
  bool kf_hit           = false;
  uint32_t kf_hit_frame = 0;
  if(hovered && !is_exporting()) {
    float ky = rect.Max.y - kKfHitR;
    for(uint32_t f : animated_frames) {
      if((int)f < *start || (int)f > *end) continue;
      float x = (float)ctx_.f2view((int)f);
      if(std::abs(mouse_x - x) <= kKfHitR && std::abs(ImGui::GetMousePos().y - ky) <= kKfHitR) {
        kf_hit       = true;
        kf_hit_frame = f;
        break;
      }
    }
  }

  bool near_left      = hovered && (mouse_x - rect.Min.x) <= kEdgeW;
  bool near_right     = hovered && (rect.Max.x - mouse_x) <= kEdgeW;
  const bool can_edit = !is_exporting();

  bool is_selected = is_selected_entt(entity.get());

  // クリックで選択(Ctrl/Shiftで追加・解除)。選択済み/選択された直後ならそのままドラッグ操作を開始する
  if(can_edit && ctx_.dragging_entt == nullptr && ctx_.dragging_kf_entt == nullptr && !ctx_.rb_active && hovered && !kf_hit && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    const bool additive = ImGui::GetIO().KeyCtrl || ImGui::GetIO().KeyShift;
    if(additive) {
      auto sel = get_selected_entts();
      auto it  = std::find(sel.begin(), sel.end(), entity);
      if(it != sel.end())
        sel.erase(it);
      else
        sel.push_back(entity);
      select_entts(sel);
    } else if(!is_selected) {
      clear_selected_entts();
      select_entt(entity);
    }
    set_selected_cache();
    is_selected = is_selected_entt(entity.get());

    if(is_selected) {
      if(near_left)
        ctx_.drag_mode = 2;
      else if(near_right)
        ctx_.drag_mode = 3;
      else
        ctx_.drag_mode = 1;
      ctx_.dragging_entt    = entity.get();
      ctx_.drag_orig_fstart = *start;
      ctx_.drag_orig_fend   = *end;
      ctx_.drag_start_frame = ctx_.view2f((int)mouse_x);

      ctx_.drag_group_orig.clear();
      if(ctx_.drag_mode == 1) {
        auto add_other = [&](Entity* other) {
          if(other == entity.get()) return;
          for(auto& t : ctx_.drag_group_orig)
            if(std::get<0>(t) == other) return;
          ctx_.drag_group_orig.push_back({other, other->fstart_, other->fend_});
        };
        if(entity->group_guid_ != 0)
          if(auto* comp = entity->get_comp())
            for(auto& other : comp->get_all_entities())
              if(other->group_guid_ == entity->group_guid_) add_other(other.get());
        for(auto* other : ctx_.selected) add_other(other); // 複数選択は一緒に動かす
      }
      collect_snap_points(entity.get());
    }
  }

  // 右クリックメニュー(未選択なら先にそのクリップだけを選択)
  if(can_edit && hovered && !kf_hit && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
    if(!is_selected) {
      clear_selected_entts();
      select_entt(entity);
      set_selected_cache();
      is_selected = true;
    }
    ctx_.ctx_entt  = entity;
    ctx_.ctx_frame = ctx_.view2f((int)mouse_x);
    ImGui::OpenPopup("tl_clip_ctx");
  }

  if(!is_exporting() && ctx_.dragging_entt == nullptr && ctx_.dragging_kf_entt == nullptr && kf_hit && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    ctx_.dragging_kf_entt   = entity.get();
    ctx_.drag_kf_orig_frame = kf_hit_frame;
  } else if(hovered && kf_hit && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
    if(entity->erase_keyframes_at(kf_hit_frame)) {
      if(auto* comp = entity->get_comp()) comp->invalidate_cache_range(*start, *end);
    }
  }

  if(ctx_.dragging_kf_entt == entity.get()) {
    if(!is_exporting() && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    } else {
      int new_frame = std::clamp(ctx_.view2f((int)ImGui::GetMousePos().x), *start, *end);
      if((uint32_t)new_frame != ctx_.drag_kf_orig_frame && entity->move_keyframes_at(ctx_.drag_kf_orig_frame, (uint32_t)new_frame)) {
        if(auto* comp = entity->get_comp()) comp->invalidate_cache_range(*start, *end);
      }
      ctx_.dragging_kf_entt = nullptr;
    }
  }

  if(ctx_.dragging_entt == entity.get()) {
    if(can_edit && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      // ワーカースレッドがrender中はentity->mtxを長時間保持するため、ここはブロックせずtry_lockする(取れなければ次フレームで再試行)
      std::unique_lock<std::mutex> lock(entity->mtx, std::try_to_lock);
      if(lock.owns_lock()) {
        int delta_f     = ctx_.view2f((int)mouse_x) - ctx_.drag_start_frame;
        auto nframes    = (int)entity->get_info().nframes; // 素材の総フレーム数(動画/音声のみ>0)
        const float ppf = (ctx_.vis_end > ctx_.vis_start) ? (float)ctx_.tl_w() / (float)(ctx_.vis_end - ctx_.vis_start) : 1.0f;
        const int thr   = std::max(1, (int)std::round(6.0f / std::max(ppf, 0.001f))); // スナップ吸着距離(約6px)
        const bool snap = ctx_.snap && !ImGui::GetIO().KeyAlt;                        // Alt押下中はスナップ無効
        ctx_.snap_line  = INT_MIN;
        int snapped     = 0;
        if(ctx_.drag_mode == 1) {
          if(snap) {
            int a, b;
            bool ha = nearest_snap(ctx_.drag_orig_fstart + delta_f, thr, &a);
            bool hb = nearest_snap(ctx_.drag_orig_fend + delta_f, thr, &b);
            int da  = ha ? a - (ctx_.drag_orig_fstart + delta_f) : 0;
            int db  = hb ? b - (ctx_.drag_orig_fend + delta_f) : 0;
            if(ha && (!hb || std::abs(da) <= std::abs(db))) {
              delta_f += da;
              ctx_.snap_line = a;
            } else if(hb) {
              delta_f += db;
              ctx_.snap_line = b;
            }
          }
          const int ns = ctx_.drag_orig_fstart + delta_f;
          const int ne = ctx_.drag_orig_fend + delta_f;
          // 移動先レイヤー(複数同時移動中は縦移動しない)。他Entityと期間が被る位置へは動けない
          auto* comp    = ctx_.active_comp;
          const int cur = ctx_.hidx;
          int tgt       = cur;
          if(comp && ctx_.drag_group_orig.empty()) tgt = std::clamp((int)std::floor((ImGui::GetMousePos().y - (ctx_.all_area.y.min + ctx_.header_h)) / std::max(1, ctx_.height)), 0, (int)comp->layers.size() - 1);
          auto is_free = [&](int layer) {
            if(!comp) return true;
            for(const auto& o : comp->layers[layer].entts) {
              if(!o || o.get() == entity.get() || !(o->fstart_ < ne && ns < o->fend_)) continue;
              bool moving = false;
              for(auto& [m, ofs, ofe] : ctx_.drag_group_orig) moving |= (m == o.get());
              if(!moving) return false;
            }
            return true;
          };
          int dst = -1;
          if(is_free(tgt))
            dst = tgt;
          else if(is_free(cur))
            dst = cur;
          if(dst >= 0) {
            *start = ns;
            *end   = ne;
            for(auto& [other, ofs, ofe] : ctx_.drag_group_orig) {
              other->fstart_ = ofs + delta_f;
              other->fend_   = ofe + delta_f;
            }
            if(dst != cur) {
              ctx_.pending_entt_move  = entity.get();
              ctx_.pending_entt_layer = dst;
            }
          }
        } else if(ctx_.drag_mode == 2) {
          int new_start = ctx_.drag_orig_fstart + delta_f;
          if(snap && nearest_snap(new_start, thr, &snapped)) {
            new_start      = snapped;
            ctx_.snap_line = snapped;
          }
          new_start = std::min(new_start, *end - 1);
          // 素材内オフセット管理は未実装のため、尺が素材の総フレーム数を超えないようclampするに留める
          if(nframes > 0 && (*end - new_start) > nframes) new_start = *end - nframes;
          *start = new_start;
        } else if(ctx_.drag_mode == 3) {
          int new_end = ctx_.drag_orig_fend + delta_f;
          if(snap && nearest_snap(new_end, thr, &snapped)) {
            new_end        = snapped;
            ctx_.snap_line = snapped;
          }
          new_end = std::max(new_end, *start + 1);
          if(nframes > 0 && (new_end - *start) > nframes) new_end = *start + nframes;
          *end = new_end;
        }
        fs   = ctx_.f2view(*start);
        fe   = ctx_.f2view(*end);
        rect = ImRect(ImVec2(fs, htop), ImVec2(fe, htop + ctx_.height));
        ImGui::SetTooltip("開始 %d  終了 %d  長さ %d f", *start, *end, *end - *start);
      }
    } else {
      if(ctx_.drag_mode == 2 || ctx_.drag_mode == 3) {
        std::lock_guard<std::mutex> lock(entity->mtx);
        // 長さ変更が確定した時点で、範囲外になった中間点の整理(ドラッグ中は連続変化するため放した時に1回だけ行う)
        // 左端ドラッグ(2)は開始位置が動いた分、右端ドラッグ(3)は末尾のみ
        entity->on_len_change_done(ctx_.drag_orig_fstart);
      }
      if(auto* comp = entity->get_comp()) {
        int f0 = std::min({ctx_.drag_orig_fstart, ctx_.drag_orig_fend, *start, *end});
        int f1 = std::max({ctx_.drag_orig_fstart, ctx_.drag_orig_fend, *start, *end});
        for(auto& [other, ofs, ofe] : ctx_.drag_group_orig) {
          f0 = std::min({f0, ofs, ofe, other->fstart_, other->fend_});
          f1 = std::max({f1, ofs, ofe, other->fstart_, other->fend_});
        }
        comp->invalidate_cache_range(f0, f1);
      }
      ctx_.dragging_entt = nullptr;
      ctx_.drag_mode     = 0;
      ctx_.snap_line     = INT_MIN;
    }
  }

  if(near_left || near_right || (ctx_.dragging_entt == entity.get() && ctx_.drag_mode >= 2)) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
  }

  // ---- 描画 ----
  auto col = entity->custom_color_ ? (ImU32)entity->custom_color_ : get_entt_color(entity);
  // 非アクティブなEntity/レイヤーはクリップを暗く表示する(斜線の網掛け付き)
  const bool dim_track = !entity->active_ || !ctx_.cur_layer_active;
  if(dim_track) {
    ImVec4 c4 = ImGui::ColorConvertU32ToFloat4(col);
    c4.w *= 0.35f;
    col = ImGui::ColorConvertFloat4ToU32(c4);
  } else if(hovered && !is_selected) {
    col = scale_color(col, 1.12f);
  } else if(is_selected) {
    col = scale_color(col, 1.25f);
  }
  auto dl     = ImGui::GetWindowDrawList();
  auto inside = ctx_.tl_area(); // レイヤー名カラムへのはみ出し描画を防ぐためこの範囲でクリップする
  dl->PushClipRect(ImVec2(inside.left(), ctx_.all_area.top()), ImVec2(inside.right(), ctx_.all_area.bottom()), true);

  // 隣接クリップと区別できるよう上下1pxの余白を取る
  ImRect body(ImVec2(rect.Min.x, rect.Min.y + 1), ImVec2(std::max(rect.Max.x, rect.Min.x + 2), rect.Max.y - 1));
  dl->AddRectFilled(body.Min, body.Max, col, 3.0f);
  dl->AddRect(body.Min, body.Max, scale_color(col, 0.55f) | 0xFF000000, 3.0f);

  if(entity->clipping_up_) { // 上のオブジェクトでクリッピング中: 上端にシアンの線
    dl->AddLine(ImVec2(body.Min.x + 2, body.Min.y + 1), ImVec2(body.Max.x - 2, body.Min.y + 1), IM_COL32(0, 230, 255, 255), 2.0f);
    if(hovered) ImGui::SetTooltip("上のオブジェクトでクリッピング中");
  }

  if(entity->camera_ctrl_) { // カメラ制御の対象: 下端にオレンジの線
    dl->AddLine(ImVec2(body.Min.x + 2, body.Max.y - 1), ImVec2(body.Max.x - 2, body.Max.y - 1), IM_COL32(255, 170, 40, 255), 2.0f);
    if(hovered) ImGui::SetTooltip("カメラ制御の対象");
  }

  if(dim_track) { // 網掛け
    dl->PushClipRect(body.Min, body.Max, true);
    for(float hx = body.Min.x - body.GetHeight(); hx < body.Max.x; hx += 6.0f) dl->AddLine(ImVec2(hx, body.Max.y), ImVec2(hx + body.GetHeight(), body.Min.y), IM_COL32(0, 0, 0, 90));
    dl->PopClipRect();
  }

  // 端ハンドル(ホバー/選択時に見えるようにする)
  if(hovered || is_selected) {
    const float hw = std::min(4.0f, body.GetWidth() / 3.0f);
    if(hw >= 2.0f) {
      const ImU32 hc = IM_COL32(255, 255, 255, (near_left || near_right) ? 130 : 70);
      dl->AddRectFilled(body.Min, ImVec2(body.Min.x + hw, body.Max.y), (near_left ? IM_COL32(255, 255, 255, 170) : hc), 3.0f, ImDrawFlags_RoundCornersLeft);
      dl->AddRectFilled(ImVec2(body.Max.x - hw, body.Min.y), body.Max, (near_right ? IM_COL32(255, 255, 255, 170) : hc), 3.0f, ImDrawFlags_RoundCornersRight);
    }
  }

  // 名前: アイコン+名前を矩形内でクリップ。画面左端で切れていても見える範囲の左端に張り付かせ、収まらなければ省略記号
  {
    const float vis_l = std::max(body.Min.x, (float)inside.left()) + 4.0f;
    const float vis_r = std::min(body.Max.x, (float)inside.right()) - 3.0f;
    const float avail = vis_r - vis_l;
    if(avail > 10.0f) {
      const std::string label = std::string(entity->clipping_up_ ? ICON_FA_CROP_SIMPLE " " : "") + (entity->camera_ctrl_ ? ICON_FA_VIDEO " " : "") + get_entt_icon(entity) + " " + name;
      const std::string shown = ellipsize(label.c_str(), avail);
      if(!shown.empty()) {
        auto tsz = ImGui::CalcTextSize(shown.c_str());
        float ty = htop + (ctx_.height - tsz.y) / 2.0f;
        dl->PushClipRect(body.Min, body.Max, true);
        dl->AddText(ImVec2(vis_l + 1, ty + 1), IM_COL32(0, 0, 0, dim_track ? 40 : 110), shown.c_str()); // 影で判読性を確保
        dl->AddText(ImVec2(vis_l, ty), dim_track ? IM_COL32(255, 255, 255, 70) : IM_COL32(255, 255, 255, 240), shown.c_str());
        dl->PopClipRect();
      }
    }
  }

  if(entity->getType() == EntityType_Audio) {
    auto* audio    = static_cast<AudioEntt*>(entity.get());
    const auto& wf = audio->waveform();
    if(!wf.levels.empty()) {
      auto* comp = entity->get_comp();
      float fps  = comp ? comp->framerate : 30.0f;
      int mid    = htop + ctx_.height / 2;
      int half   = ctx_.height / 2 - 1;
      for(int x = (int)rect.Min.x; x < (int)rect.Max.x; x++) {
        int f = ctx_.view2f(x);
        if(f < *start || f >= *end) continue;
        int idx = wf.index_for_second((double)audio->offset_sec_ + (f - *start) / (double)fps * (audio->speed / 100.0));
        if(idx < 0 || idx >= (int)wf.levels.size()) continue;
        int len = (int)(half * (wf.levels[idx] / 255.0f));
        if(len > 0) dl->AddLine(ImVec2((float)x, (float)(mid - len)), ImVec2((float)x, (float)(mid + len)), IM_COL32(255, 255, 255, 200)); // トラック背景(緑系)とのコントラストを確保するため白系に
      }
    }
  }

  { // 中間点(キーフレーム)の集約表示: Entity帯下端にダイヤを重ね描きする(AviUtl方式、frame単位で全プロパティ横断)
    float ky = body.Max.y - kKfHitR;
    for(uint32_t f : animated_frames) {
      if((int)f < *start || (int)f > *end) continue;
      float x    = (float)ctx_.f2view((int)f);
      bool cur   = ctx_.dragging_kf_entt == entity.get() && f == ctx_.drag_kf_orig_frame;
      ImU32 kcol = cur ? IM_COL32(255, 255, 255, 255) : IM_COL32(255, 190, 40, 255);
      dl->AddQuadFilled(ImVec2(x, ky - kKfHitR), ImVec2(x + kKfHitR, ky), ImVec2(x, ky + kKfHitR), ImVec2(x - kKfHitR, ky), kcol);
    }
  }


  // 選択の強調枠
  if(is_selected) dl->AddRect(body.Min, body.Max, IM_COL32(255, 235, 130, 255), 3.0f, 0, 2.0f);
  dl->PopClipRect();

  // ホバーのツールチップ(ドラッグ中・メニュー表示中は出さない)
  if(hovered && ctx_.dragging_entt == nullptr && !ImGui::IsPopupOpen("tl_clip_ctx")) {
    const float fps = ctx_.fps > 0 ? ctx_.fps : 30.0f;
    ImGui::SetTooltip("%s\n種別: %s%s\n開始 %d  終了 %d  長さ %d f (%.2f 秒)", name, entt_type_label(entity.get()), entity->active_ ? "" : "  (無効)", *start, *end, *end - *start, (*end - *start) / fps);
  }
  return hovered;
}

void EndTrack() {}

void SetTimelineRightStripWidth(int w) { ctx_.right_strip_w = std::max(0, w); }

void SetTimelineViewRange(FrameT start, FrameT end) {
  ctx_.vis_start = start;
  ctx_.vis_end   = end;
}

bool ConsumeTimelineFitRequest() {
  bool v           = ctx_.pending_fit;
  ctx_.pending_fit = false;
  return v;
}

void RequestTimelineFit() { ctx_.pending_fit = true; }

const char* GetTimelineLayerSearch() { return ctx_.layer_search_buf; }

void ResetTimelineState() { ctx_ = TimelineContext(); }

} // namespace mu
