#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif

#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/core/rect.hpp>
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
  Rect all_area;
  int hidx = 0;
  int trackname_width = 100;
  bool toggle_play = false;
  int visible_start = 0;
  int height = 16;
  int lasy_mouse_x = 0;
  Entity* last_entt_hov = nullptr;
  int header_h = 20;
  int visible_end = 100;
  bool hide_tabbar = false;
  int cur_frame = 0;
  bool first = true;
  std::vector<Entity*> sel; // TODO: 複数選択を可能にする

  Rect tl_area() {
    auto r = all_area;
    r.y.min += header_h;
    r.x.min += trackname_width;
    return r;
  }

  Rect header_area() {
    auto r = all_area;
    r.y.max = r.y.min + header_h;
    return r;
  }

  int f2view(FrameT f) {
    if(visible_start >= visible_end) return visible_start;
    auto d = f - visible_start;
    return tl_area().x.min + tl_area().w() * d / (visible_end - visible_start);
  }
  int view2f(int x) {
    auto d = x - tl_area().x.min;
    return visible_start + (visible_end - visible_start) * d / tl_area().w();
  }
};

static TimelineContext ctx_;
ImTimelineColors col_;


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

bool BeginTimeline(const char* name, FrameT* frame, FrameT* start, FrameT* end, bool* playing, const ImVec2& size) {
  if(playing != nullptr && ctx_.toggle_play) *playing = !*playing;
  ctx_.toggle_play = false;

  // available max height
  {
    auto height = ImGui::GetContentRegionAvail().y;
    auto window = ImGui::GetCurrentWindow();
    auto width_ = std::max(size.x, window->InnerClipRect.GetWidth());
    ctx_.height = ImGui::GetTextLineHeightWithSpacing();
    auto height_ = std::max<int>(height, ctx_.hidx * ctx_.height);
    auto pos = ImGui::GetCursorScreenPos();

    ctx_.all_area = Rect(pos.x, pos.x + width_, pos.y, pos.y + height_);
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
  {
    auto width = ImGui::GetTextLineHeightWithSpacing();
    ImRect re(all.x.min, all.y.min, all.x.min + width, all.y.min + width);
    auto hovered = ImGui::IsMouseHoveringRect(re.Min, re.Max);
    auto bg = ImGui::GetStyle().Colors[hovered ? ImGuiCol_ButtonHovered : ImGuiCol_ButtonActive];
    auto col = IM_COL32(bg.x * 255, bg.y * 255, bg.z * 255, bg.w * 255);
    dl->AddRectFilled(re.Min, re.Max, col);
    dl->AddRect(re.Min, re.Max, col_.border);
    /*dl->AddText(re.Min, IM_COL32(255, 255, 255, 255), (ctx_.locked ? ICON_FA_LOCK : ICON_FA_UNLOCK));*/
    /*if(ImGui::IsMouseClicked(ImGuiMouseButton_Left) && hovered && ImGui::IsWindowHovered()) ctx_.locked = !ctx_.locked;*/
  }

  auto bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
  dl->AddRectFilled(ImVec2(all.left() + ctx_.trackname_width, all.top()), ImVec2(all.right(), all.bottom()), IM_COL32(bg.x * 255, bg.y * 255, bg.z * 255, bg.w * 255));

  {
    auto area = all;
    area.x.max = area.x.min + ctx_.trackname_width;
    auto col = IM_COL32(0, 0, 0, 100);
    dl->AddRectFilled(ImVec2(area.left(), area.top() + item_height), ImVec2(area.right(), area.bottom()), col);
  }

  dl->AddRect(ImVec2(all.left(), all.top()), ImVec2(all.right(), all.bottom()), col_.border);

  if(ctx_.first) {
    ctx_.visible_start = *start - 20;
    ctx_.visible_end = *end + 20;
    ctx_.first = false;
  }

  // draw header
  {
    auto he = ctx_.header_area();
    dl->AddRectFilled(ImVec2(he.x.min, he.y.min), ImVec2(he.x.max, he.y.max), col_.header_bg);
    dl->AddRect(ImVec2(he.x.min, he.y.min), ImVec2(he.x.max, he.y.max), col_.border);

    int di = 10;
    if(he.w() > 1) {
      float pi = (ctx_.visible_end - ctx_.visible_start) / he.w();
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
    for(FrameT i = (ctx_.visible_start / di) * di; i < ctx_.visible_end; i += di) {
      auto x = ctx_.f2view(i);
      dl->AddLine(ImVec2(x, ctx_.all_area.y.min), ImVec2(x, ctx_.all_area.y.min + 20), IM_COL32(255, 255, 255, 100));
      dl->AddText(ImVec2(x, ctx_.all_area.y.min), IM_COL32(255, 255, 255, 100), std::to_string(i).c_str());
    }

    int di2 = std::max(1, di / 10);
    for(FrameT i = ctx_.visible_start; i < ctx_.visible_end; i += di2) {
      auto x = ctx_.f2view(i);
      dl->AddLine(ImVec2(x, ctx_.all_area.y.min), ImVec2(x, ctx_.all_area.y.min + 8), IM_COL32(255, 255, 255, 50));
    }

    // Compositionの範囲を描画
    int st = ctx_.f2view(*start);
    int ed = ctx_.f2view(*end);
    st = std::clamp<int>(st, he.x.min, he.x.max);
    ed = std::clamp<int>(ed, he.x.min, he.x.max);
    dl->AddRectFilled(ImVec2(st, he.y.max - 8), ImVec2(ed, he.y.max), IM_COL32(0, 150, 255, 100));

    // Compositionのスタートゴールを描画し、<kbd>[</kbd>と<kbd>]</kbd>キーで終端を設定
    ImRect comp_start_ = ImRect(ImVec2(st - 2, he.y.min), ImVec2(st + 2, he.y.max));
    ImRect comp_end_ = ImRect(ImVec2(ed - 2, he.y.min), ImVec2(ed + 2, he.y.max));

    bool start_hovered = ImGui::IsMouseHoveringRect(comp_start_.Min, comp_start_.Max);
    bool end_hovered = ImGui::IsMouseHoveringRect(comp_end_.Min, comp_end_.Max);
    if(start_hovered) ImGui::SetTooltip("スタートフレーム=%d", *start);
    if(end_hovered) ImGui::SetTooltip("エンドフレーム=%d", *end);
    dl->AddRectFilled(comp_start_.Min, comp_start_.Max, IM_COL32(0, 180, 255, start_hovered ? 255 : 200));
    dl->AddRectFilled(comp_end_.Min, comp_end_.Max, IM_COL32(0, 180, 255, end_hovered ? 255 : 200));
  }


  // 現在のフレームのバーを描画
  {
    auto x = ctx_.f2view(*frame);

    constexpr int scl_w = 5; // 現在フレームを移動させるバーの幅
    ImVec2 p1(x - scl_w, ctx_.all_area.y.min);
    auto H = ImGui::GetTextLineHeightWithSpacing();
    ImVec2 p2(x + scl_w, ctx_.all_area.y.min + H);
    auto col = IM_COL32(170, 0, 0, 200);
    if(ImGui::IsMouseHoveringRect(p1, p2)) col = IM_COL32(255, 0, 0, 255);
    dl->AddRectFilled(p1, p2, col);
    dl->AddLine(ImVec2(x, ctx_.all_area.y.min), ImVec2(x, ctx_.all_area.bottom()), col);

    auto h_ = ctx_.header_area();
    bool in_header_area = ImGui::IsMouseHoveringRect(ImVec2(h_.x.min, h_.y.min), ImVec2(h_.x.max, h_.y.max));
    bool lclick = ImGui::IsMouseClicked(ImGuiMouseButton_Left) || ImGui::IsMouseDragging(ImGuiMouseButton_Left);
    if(in_header_area && lclick) {
      // ctx_.state = Draginctx_Cursor;
      *frame = ctx_.view2f(ImGui::GetMousePos().x);
    }
  }

  ctx_.hidx = 1;
  ctx_.cur_frame = *frame;
  ctx_.visible_start = *start;
  ctx_.visible_end = *end;
  return open;
}

int EndTimeline() {
  // scrolling
  if(ImGui::IsWindowHovered()) {
    if(ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
      auto delta = ImGui::GetIO().MouseDelta.x;
      ctx_.visible_start -= delta;
      ctx_.visible_end -= delta;
    }

    // 拡大縮小(マウスホイール)
    {
      auto delta = ImGui::GetIO().MouseWheel;
      auto center = ctx_.view2f(ImGui::GetMousePos().x);
      auto scale = 1.0f + delta / 15.0f;
      ctx_.visible_start = center + (ctx_.visible_start - center) * scale;
      ctx_.visible_end = center + (ctx_.visible_end - center) * scale;
    }
    return ctx_.cur_frame;
  }

  if(ctx_.cur_frame && ctx_.visible_start && ctx_.visible_end) {
    ctx_.cur_frame = std::clamp<int>(ctx_.cur_frame, ctx_.visible_start, ctx_.visible_end);
  }

  auto dl = ImGui::GetWindowDrawList();
  auto all = ctx_.all_area;
  auto line_col = ImGui::GetStyle().Colors[ImGuiCol_Border];
  ImVec2 p1(all.left(), all.top());
  ImVec2 p2(all.right(), all.top());
  int header_height = ImGui::GetTextLineHeightWithSpacing();
  for(int i = 0; i < ctx_.hidx + 1; i++) {
    dl->AddLine(p1, p2, IM_COL32(line_col.x * 255, line_col.y * 255, line_col.z * 255, line_col.w * 255), 1.5);
    p1.y += header_height;
    p2.y += header_height;
  }

  bool open_config = false;
  {
    ImGui::SetCursorScreenPos(ImVec2(all.left(), all.top()));
    /*auto str = ctx_.playing ? ICON_FA_PAUSE : ICON_FA_PLAY;*/
    /*if(wd_checkbox_txt(str, &ctx_.locked)) ctx_.toggle_play = true;*/
    if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(1)) open_config = true;

    ImGui::SameLine();
    ImGui::SetNextItemWidth(60);
    ImGui::DragInt("## frame", &ctx_.cur_frame);
    ImGui::SameLine();
    ImGui::Text("/");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    int range[2] = {ctx_.visible_start, ctx_.visible_end};
    ImGui::DragInt2("## start", range);
    ctx_.visible_start = range[0];
    ctx_.visible_end = range[1];
  }

  ImGui::EndChild();


  if(open_config) ImGui::OpenPopup("Timeline Config");
  if(ImGui::BeginPopup("Timeline Config")) {
    /*ImGui::Checkbox(ICON_FA_LOCK " Lock", &ctx_.locked);*/
    if(ImGui::Button("タブバー表示を切り替え")) ctx_.hide_tabbar = !ctx_.hide_tabbar;
    ImGui::EndPopup();
  }
  /*ctx_.last_mouse_x = ImGui::GetMousePos().x;*/
  /*if(!ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {*/
  /*  ctx_.state = None;*/
  /*  ctx_.selected.clear();*/
  /*}*/
}

bool BeginLayer(TrackLayer* layer) {
  auto dl = ImGui::GetWindowDrawList();
  auto inside = ctx_.tl_area();

  int x = inside.left() + ImGui::GetStyle().ItemSpacing.x;

  float clipre[4] = {x, inside.top() + ctx_.height * ctx_.hidx, inside.right(), inside.top() + ctx_.height * (ctx_.hidx + 1)};
  if(ImGui::IsMouseHoveringRect(ImVec2(clipre[0], clipre[1]), ImVec2(clipre[2], clipre[3]))) //
    ImGui::SetTooltip("name=%s", layer->name);

  ImRect R(ImVec2(inside.left(), inside.top() + ctx_.height * ctx_.hidx), ImVec2(inside.right(), inside.top() + ctx_.height * (ctx_.hidx + 1)));
  bool line_hovered = ImGui::IsMouseHoveringRect(R.Min, R.Max);
  if(line_hovered) dl->AddRectFilled(R.Min, R.Max, IM_COL32(255, 255, 255, 20));

  ImRect sidebar(ImVec2(ctx_.all_area.left(), inside.top() + ctx_.height * ctx_.hidx), //
                 ImVec2(inside.left() + ctx_.trackname_width, inside.top() + ctx_.height * (ctx_.hidx + 1)));
  dl->AddRectFilled(sidebar.Min, sidebar.Max, IM_COL32(40, 40, 40, 255));
  dl->AddText(ImVec2(x, ctx_.all_area.top() + ctx_.height * ctx_.hidx), IM_COL32(255, 255, 255, 100), layer->name);
  return true;
}

void EndLayer() {
  ctx_.hidx++;
}

bool IsTimeline_LineHovered() {
  auto dh = ImGui::GetTextLineHeightWithSpacing();
  auto h = dh * (ctx_.hidx - 1);
  auto all = ctx_.all_area;
  all.y = all.y.shift(h);
  all.y.max = all.y.min + dh;

  ImRect rect(ImVec2(all.left(), all.top()), ImVec2(all.left() + ctx_.prop_view_width, all.top() + dh));
  return ImGui::IsMouseHoveringRect(rect.Min, rect.Max);
}

bool IsTimelineKeyHovered() {
  return ctx_.last_entt_hov;
}

bool IsTimelineClickedLeftButton() {
  return ctx_.last_entt_hov;
}

bool BeginTrack(const Ref<Entity>& entity) {
  MU_ASSERT(entity);
  const char* name = entity->name;
  int* start = &entity->trk.fstart;
  int* end = &entity->trk.fend;
  int htop = ctx_.tl_area().y.min + ctx_.height * ctx_.hidx;

  auto col = IM_COL32(255, 0, 0, 100);
  auto dl = ImGui::GetWindowDrawList();
  int fs = ctx_.f2view(*start);
  int fe = ctx_.f2view(*end);
  ImRect rect(ImVec2(fs, htop), ImVec2(fe, htop + ctx_.height));
  bool hovered = ImGui::IsMouseHoveringRect(rect.Min, rect.Max);
  if(hovered)
    ctx_.last_entt_hov = entity.get();
  else
    ctx_.last_entt_hov = nullptr;

  dl->AddRect(rect.Min, rect.Max, col_.border);
  dl->AddRectFilled(rect.Min, rect.Max, col);
  dl->AddText(ImVec2(fs, htop), IM_COL32(255, 255, 255, 100), name);
  return hovered;
}

void EndTrack() {}

bool ShouldHideTimelineTabBar() {
  return ctx_.hide_tabbar;
}

void SetTimelineViewRange(FrameT start, FrameT end) {
  ctx_.visible_start = start;
  ctx_.visible_end = end;
}

void ResetTimelineState() {
  ctx_ = TimelineContext();
}

} // namespace mu
