#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif

#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cutil/rect.hpp>
#include <string>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/app/export_state.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/timeline.hpp>

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
  int cur_frame         = 0;
  bool first            = true;
  bool cur_layer_active = true; // BeginLayerで設定し、そのレイヤー内のBeginTrackが参照する
  std::vector<Entity*> sel; // TODO: 複数選択を可能にする

  // レイヤー名インライン編集
  int editing_layer_idx      = -1;
  char editing_layer_buf[64] = {0};

  // ヘッダー左端(フィット/タイムコード/検索)
  bool pending_fit           = false;
  bool search_open           = false;
  char layer_search_buf[64]  = {0};

  // レイヤーの削除/移動は破壊的操作のためEndTimeline()側で遅延適用する
  Composition* active_comp = nullptr;
  int pending_delete_layer = -1;
  int pending_move_layer   = -1;
  int pending_move_dir     = 0; // -1: 上へ, +1: 下へ

  // クリップのドラッグ移動/端リサイズ
  Entity* dragging_entt = nullptr;
  int drag_mode         = 0; // 0=none, 1=move, 2=resize_left, 3=resize_right
  int drag_orig_fstart  = 0;
  int drag_orig_fend    = 0;
  int drag_start_frame  = 0;

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

  // available max height
  {
    auto height  = ImGui::GetContentRegionAvail().y;
    auto window  = ImGui::GetCurrentWindow();
    auto width_  = std::max(size.x, window->InnerClipRect.GetWidth());
    ctx_.height  = ImGui::GetTextLineHeightWithSpacing();
    auto height_ = std::max<int>(height, ctx_.hidx * ctx_.height);
    auto pos     = ImGui::GetCursorScreenPos();

    ctx_.all_area = cutil::Rect(pos.x, pos.x + width_, pos.y, pos.y + height_);
  }

  auto all = ctx_.all_area;

  bool open;
  const float item_height = ImGui::GetTextLineHeightWithSpacing();

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
    dl->AddRectFilled(ImVec2(area.left(), area.top() + item_height), ImVec2(area.right(), area.bottom()), col);
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

    int di = 10;
    if(he.w() > 1) {
      float pi = (ctx_.vis_end - ctx_.vis_start) / he.w();
      if(pi < 0.1)
        di = 10;
      else if(pi < 1.1)
        di = 50;
      else if(pi < 3.2)
        di = 100;
      else if(pi < 5.4)
        di = 200;
      else
        di = 500;
    }
    for(FrameT i = (ctx_.vis_start / di) * di; i < ctx_.vis_end; i += di) {
      auto x = ctx_.f2view(i);
      dl->AddLine(ImVec2(x, ctx_.all_area.y.min), ImVec2(x, ctx_.all_area.y.min + 20), IM_COL32(255, 255, 255, 100));
      dl->AddText(ImVec2(x, ctx_.all_area.y.min), IM_COL32(255, 255, 255, 100), std::to_string(i).c_str());
    }

    int di2 = std::max(1, di / 10);
    for(FrameT i = ctx_.vis_start; i < ctx_.vis_end; i += di2) {
      auto x = ctx_.f2view(i);
      dl->AddLine(ImVec2(x, ctx_.all_area.y.min), ImVec2(x, ctx_.all_area.y.min + 8), IM_COL32(255, 255, 255, 50));
    }

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
    if(start_hovered) ImGui::SetTooltip("スタートフレーム=%d", *start);
    if(end_hovered) ImGui::SetTooltip("エンドフレーム=%d", *end);
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

    constexpr int scl_w = 5; // 現在フレームを移動させるバーの幅
    ImVec2 p1(x - scl_w, ctx_.all_area.y.min);
    auto H = ImGui::GetTextLineHeightWithSpacing();
    ImVec2 p2(x + scl_w, ctx_.all_area.y.min + H);
    auto col = IM_COL32(170, 0, 0, 200);
    if(ImGui::IsMouseHoveringRect(p1, p2)) col = IM_COL32(255, 0, 0, 255);
    // トラック名カラム(サイドバー)へのはみ出しを防ぐ
    dl->PushClipRect(ImVec2(all.left() + ctx_.trackname_width, all.top()), ImVec2(all.right(), all.bottom()), true);
    dl->AddRectFilled(p1, p2, col);
    dl->AddLine(ImVec2(x, ctx_.all_area.y.min), ImVec2(x, ctx_.all_area.bottom()), col);
    dl->PopClipRect();

    auto h_             = ctx_.header_area();
    bool in_header_area = ImGui::IsMouseHoveringRect(ImVec2(h_.x.min, h_.y.min), ImVec2(h_.x.max, h_.y.max));
    bool lclick         = ImGui::IsMouseClicked(ImGuiMouseButton_Left) || ImGui::IsMouseDragging(ImGuiMouseButton_Left);
    if(in_header_area && lclick) {
      *frame = ctx_.view2f(ImGui::GetMousePos().x);
    }
  }

  ctx_.hidx      = 0;
  ctx_.cur_frame = *frame;
  // vis_start/vis_endはズーム/パン状態のため*start/*end(Composition境界)で上書きしない(上書きするとズームが毎フレーム巻き戻る)
  return open;
}

int EndTimeline() {
  int return_value = ctx_.cur_frame;
  if(ImGui::IsWindowHovered()) {
    if(ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
      auto delta = ImGui::GetIO().MouseDelta.x;
      ctx_.vis_start -= delta;
      ctx_.vis_end -= delta;
    }

    // マウスホイール: Shift押下時はパン、それ以外はズーム
    {
      auto delta = ImGui::GetIO().MouseWheel;
      if(ImGui::GetIO().KeyShift) {
        auto pan_amount = (ctx_.vis_end - ctx_.vis_start) * (-delta) / 20.0f;
        ctx_.vis_start += pan_amount;
        ctx_.vis_end += pan_amount;
      } else {
        auto center    = ctx_.view2f(ImGui::GetMousePos().x);
        auto scale     = 1.0f + delta / 15.0f;
        ctx_.vis_start = center + (ctx_.vis_start - center) * scale;
        ctx_.vis_end   = center + (ctx_.vis_end - center) * scale;
      }
    }
    return_value = ctx_.cur_frame;
  }

  // レイヤーの削除/移動を遅延適用(ループ中のvector破壊を避けるため)
  if(ctx_.active_comp) {
    auto* cp     = ctx_.active_comp;
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(cp->mtx);
      if(ctx_.pending_delete_layer >= 0 && ctx_.pending_delete_layer < (int)cp->layers.size()) {
        cp->layers.erase(cp->layers.begin() + ctx_.pending_delete_layer);
        ctx_.pending_delete_layer = -1;
        changed                   = true;
      }
      if(ctx_.pending_move_layer >= 0 && ctx_.pending_move_layer < (int)cp->layers.size()) {
        int i = ctx_.pending_move_layer;
        int j = i + ctx_.pending_move_dir;
        if(j >= 0 && j < (int)cp->layers.size()) std::swap(cp->layers[i], cp->layers[j]);
        ctx_.pending_move_layer = -1;
        changed                 = true;
      }
    }
    if(changed) cp->cache.invalidate_all();
  }

  if(ctx_.cur_frame && ctx_.vis_start && ctx_.vis_end) {
    ctx_.cur_frame = std::clamp<int>(ctx_.cur_frame, ctx_.vis_start, ctx_.vis_end);
  }

  auto dl       = ImGui::GetWindowDrawList();
  auto all      = ctx_.all_area;
  auto line_col = ImGui::GetStyle().Colors[ImGuiCol_Border];
  ImVec2 p1(all.left(), all.y.min + ctx_.header_h);
  ImVec2 p2(all.right(), all.y.min + ctx_.header_h);
  int header_height = ImGui::GetTextLineHeightWithSpacing();
  for(int i = 0; i < ctx_.hidx + 1; i++) {
    dl->AddLine(p1, p2, IM_COL32(line_col.x * 255, line_col.y * 255, line_col.z * 255, line_col.w * 255), 1.5);
    p1.y += header_height;
    p2.y += header_height;
  }

  ImGui::EndChild();
  return return_value;
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

  int eye_w  = hbtm - htop; // 目アイコン用の正方形幅(行高さに合わせる)
  int name_x = x + eye_w + 2;

  ImRect sidebar(ImVec2(x, htop), ImVec2(inside.left(), hbtm));
  ImRect eye_rect(ImVec2(x, htop), ImVec2(x + eye_w, hbtm));
  bool eye_hovered     = ImGui::IsMouseHoveringRect(eye_rect.Min, eye_rect.Max);
  bool sidebar_hovered = ImGui::IsMouseHoveringRect(sidebar.Min, sidebar.Max) && !eye_hovered;

  if(!is_exporting() && eye_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) layer->active = !layer->active;
  if(eye_hovered) ImGui::SetTooltip(layer->active ? "レイヤーを非表示にする" : "レイヤーを表示する");

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
    if(sidebar_hovered) ImGui::SetTooltip("name=%s", layer->name.c_str());
    if(!is_exporting() && sidebar_hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
      ctx_.editing_layer_idx = layer_idx;
      std::snprintf(ctx_.editing_layer_buf, sizeof(ctx_.editing_layer_buf), "%s", layer->name.c_str());
    }
  }

  // レイヤーにマウスが載っていたらlayer全体をハイライトする
  ImRect R(ImVec2(inside.left(), htop), ImVec2(inside.right(), hbtm));
  bool line_hovered = ImGui::IsMouseHoveringRect(R.Min, R.Max);
  if(line_hovered) dl->AddRectFilled(R.Min, R.Max, IM_COL32(255, 255, 255, 20));

  dl->AddRectFilled(sidebar.Min, sidebar.Max, layer->active ? IM_COL32(40, 40, 40, 255) : IM_COL32(20, 20, 20, 255));
  if(!layer->active) dl->AddRectFilled(ImVec2(inside.left(), htop), ImVec2(inside.right(), hbtm), IM_COL32(0, 0, 0, 110)); // 非表示レイヤーはトラック部分も暗くする
  dl->AddText(ImVec2(x + 2, htop + (eye_w - ImGui::GetTextLineHeight()) / 2), layer->active ? IM_COL32(255, 255, 255, 200) : IM_COL32(255, 255, 255, 80), layer->active ? ICON_FA_EYE : ICON_FA_EYE_SLASH);

  if(!editing) {
    const char* search = ctx_.layer_search_buf;
    bool dim           = !layer->active || (search[0] != '\0' && !strstr(layer->name.c_str(), search));
    auto tsz           = ImGui::CalcTextSize(layer->name.c_str());
    float ty           = htop + ((hbtm - htop) - tsz.y) / 2.0f;
    dl->AddText(ImVec2(name_x, ty), dim ? IM_COL32(255, 255, 255, 40) : IM_COL32(255, 255, 255, 100), layer->name.c_str());
  }

  if(!is_exporting() && sidebar_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) ImGui::OpenPopup(("layer_ctx_" + std::to_string(layer_idx)).c_str());
  if(ImGui::BeginPopup(("layer_ctx_" + std::to_string(layer_idx)).c_str())) {
    if(ImGui::MenuItem("削除")) ctx_.pending_delete_layer = layer_idx;
    if(ImGui::MenuItem("上へ移動")) {
      ctx_.pending_move_layer = layer_idx;
      ctx_.pending_move_dir   = -1;
    }
    if(ImGui::MenuItem("下へ移動")) {
      ctx_.pending_move_layer = layer_idx;
      ctx_.pending_move_dir   = 1;
    }
    ImGui::EndPopup();
  }
  return true;
}

void EndLayer() { ctx_.hidx++; }

bool IsTimeline_LineHovered() {
  auto dh   = ImGui::GetTextLineHeightWithSpacing();
  auto h    = dh * (ctx_.hidx - 1);
  auto all  = ctx_.all_area;
  all.y     = all.y.shift(h);
  all.y.max = all.y.min + dh;

  ImRect rect(ImVec2(all.left(), all.top()), ImVec2(all.left() + ctx_.trackname_width, all.top() + dh));
  return ImGui::IsMouseHoveringRect(rect.Min, rect.Max);
}

bool IsTimelineKeyHovered() { return ctx_.last_entt_hov; }

bool IsTimelineClickedLeftButton() { return ctx_.last_entt_hov; }

bool BeginTrack(const Ref<Entity>& entity) {
  MU_ASSERT(entity);
  const char* name = entity->name.c_str();
  int* start       = &entity->trk.fstart;
  int* end         = &entity->trk.fend;
  int htop         = ctx_.layer_y1();

  constexpr int kEdgeW = 5; // 左右端のドラッグ判定幅(px)

  int fs = ctx_.f2view(*start);
  int fe = ctx_.f2view(*end);
  ImRect rect(ImVec2(fs, htop), ImVec2(fe, htop + ctx_.height));
  bool hovered = ImGui::IsMouseHoveringRect(rect.Min, rect.Max);
  if(hovered)
    ctx_.last_entt_hov = entity.get();
  else
    ctx_.last_entt_hov = nullptr;

  auto mouse_x    = ImGui::GetMousePos().x;
  bool near_left  = hovered && (mouse_x - rect.Min.x) <= kEdgeW;
  bool near_right = hovered && (rect.Max.x - mouse_x) <= kEdgeW;

  bool is_selected = false;
  for(const auto& e : get_selected_entts()) {
    if(e.get() == entity.get()) {
      is_selected = true;
      break;
    }
  }

  if(!is_exporting() && ctx_.dragging_entt == nullptr && hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    if(near_left)
      ctx_.drag_mode = 2;
    else if(near_right)
      ctx_.drag_mode = 3;
    else if(is_selected)
      ctx_.drag_mode = 1;
    if(ctx_.drag_mode != 0) {
      ctx_.dragging_entt    = entity.get();
      ctx_.drag_orig_fstart = *start;
      ctx_.drag_orig_fend   = *end;
      ctx_.drag_start_frame = ctx_.view2f((int)mouse_x);
    }
  }

  if(ctx_.dragging_entt == entity.get()) {
    if(!is_exporting() && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      // ワーカースレッドがrender中はentity->mtxを長時間保持するため、ここはブロックせずtry_lockする(取れなければ次フレームで再試行)
      std::unique_lock<std::mutex> lock(entity->mtx, std::try_to_lock);
      if(lock.owns_lock()) {
        int delta_f  = ctx_.view2f((int)mouse_x) - ctx_.drag_start_frame;
        auto nframes = (int)entity->get_info().nframes; // 素材の総フレーム数(動画/音声のみ>0)
        if(ctx_.drag_mode == 1) {
          *start = ctx_.drag_orig_fstart + delta_f;
          *end   = ctx_.drag_orig_fend + delta_f;
        } else if(ctx_.drag_mode == 2) {
          int new_start = std::min(ctx_.drag_orig_fstart + delta_f, *end - 1);
          // 素材内オフセット管理は未実装のため、尺が素材の総フレーム数を超えないようclampするに留める
          if(nframes > 0 && (*end - new_start) > nframes) new_start = *end - nframes;
          *start = new_start;
        } else if(ctx_.drag_mode == 3) {
          int new_end = std::max(ctx_.drag_orig_fend + delta_f, *start + 1);
          if(nframes > 0 && (new_end - *start) > nframes) new_end = *start + nframes;
          *end = new_end;
        }
        fs   = ctx_.f2view(*start);
        fe   = ctx_.f2view(*end);
        rect = ImRect(ImVec2(fs, htop), ImVec2(fe, htop + ctx_.height));
      }
    } else {
      if(auto* comp = entity->get_comp()) {
        int f0 = std::min({ctx_.drag_orig_fstart, ctx_.drag_orig_fend, *start, *end});
        int f1 = std::max({ctx_.drag_orig_fstart, ctx_.drag_orig_fend, *start, *end});
        comp->cache.invalidate_range(f0, f1);
      }
      ctx_.dragging_entt = nullptr;
      ctx_.drag_mode     = 0;
    }
  }

  if(near_left || near_right || (ctx_.dragging_entt == entity.get() && ctx_.drag_mode >= 2)) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
  }

  auto col = entity->trk.custom_color ? (ImU32)entity->trk.custom_color : get_entt_color(entity);
  // 非アクティブなEntity/レイヤーはクリップを暗く表示する
  bool dim_track = !entity->trk.active_ || !ctx_.cur_layer_active;
  if(dim_track) {
    ImVec4 c4 = ImGui::ColorConvertU32ToFloat4(col);
    c4.w *= 0.35f;
    col = ImGui::ColorConvertFloat4ToU32(c4);
  }
  auto dl     = ImGui::GetWindowDrawList();
  auto inside = ctx_.tl_area(); // レイヤー名カラムへのはみ出し描画を防ぐためこの範囲でクリップする
  dl->PushClipRect(ImVec2(inside.left(), ctx_.all_area.top()), ImVec2(inside.right(), ctx_.all_area.bottom()), true);
  dl->AddRect(rect.Min, rect.Max, col_.border);
  dl->AddRectFilled(rect.Min, rect.Max, col);
  {
    auto tsz = ImGui::CalcTextSize(name);
    float ty = htop + (ctx_.height - tsz.y) / 2.0f;
    dl->AddText(ImVec2(fs + 2, ty), dim_track ? IM_COL32(255, 255, 255, 40) : IM_COL32(255, 255, 255, 100), name);
  }
  dl->PopClipRect();
  return hovered;
}

void EndTrack() {}

void SetTimelineViewRange(FrameT start, FrameT end) {
  ctx_.vis_start = start;
  ctx_.vis_end   = end;
}

bool ConsumeTimelineFitRequest() {
  bool v         = ctx_.pending_fit;
  ctx_.pending_fit = false;
  return v;
}

const char* GetTimelineLayerSearch() { return ctx_.layer_search_buf; }

void ResetTimelineState() { ctx_ = TimelineContext(); }

} // namespace mu
