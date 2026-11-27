#include <IconsFontAwesome6.h>
#include <algorithm>
#include <movutl/asset/image.hpp>
#include <movutl/graphics/GLTexture.hpp>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {

struct NavigatorPanel::Texture {
  GLTexture gl;
  Ref<Image> img;
  uint64_t revision = 0; // 文書をまたいで重複しない
  double updated    = -1;
};

namespace {

constexpr int kThumbSize = 256; // 縮小画像の長辺

// ponytail: 縮小画像は1画素あたり2x2点をpick()するだけ(細線はかすれる)。重い・粗いと感じたらタイル単位の縮小キャッシュにする
void make_thumbnail(const PaintDocument& d, Image& out) {
  const float sx = (float)d.width() / out.width, sy = (float)d.height() / out.height;
  for(unsigned y = 0; y < out.height; y++) {
    for(unsigned x = 0; x < out.width; x++) {
      int sum[4] = {};
      for(int j = 0; j < 4; j++) {
        const Vec4b c = d.pick((int)((x + 0.25f + 0.5f * (j % 2)) * sx), (int)((y + 0.25f + 0.5f * (j / 2)) * sy));
        for(int k = 0; k < 3; k++) sum[k] += c[k] * c[3];
        sum[3] += c[3];
      }
      Vec4b v;
      for(int k = 0; k < 3; k++) v[k] = sum[3] ? (unsigned char)(sum[k] / sum[3]) : 0;
      v[3] = (unsigned char)(sum[3] / 4);
      out.set_rgba(x, y, v);
    }
  }
}

} // namespace

void NavigatorPanel::Update() {
  if(!ImGui::Begin(paint_window_title("navigator").c_str(), &open)) {
    ImGui::End();
    return;
  }
  auto& st                 = paint_editor();
  const PaintDocument* doc = st.doc();
  if(!doc) {
    ImGui::TextDisabled("キャンバスがありません");
    ImGui::End();
    return;
  }
  if(!tex_) tex_ = std::make_shared<Texture>();
  const float scale = (float)kThumbSize / std::max(doc->width(), doc->height());
  const int w = std::max(1, (int)(doc->width() * scale)), h = std::max(1, (int)(doc->height() * scale));
  // 描画中は更新しない。連続する変更(スライダー等)は間引く
  if(tex_->revision != doc->revision() && !doc->stroking() && ImGui::GetTime() - tex_->updated > 0.15) {
    if(!tex_->img || (int)tex_->img->width != w || (int)tex_->img->height != h) tex_->img = cutil::make_ref<Image>(w, h);
    make_thumbnail(*doc, *tex_->img);
    tex_->gl.set(tex_->img);
    tex_->revision = doc->revision();
    tex_->updated  = ImGui::GetTime();
  }

  CanvasView& view = st.view;
  // キャンバスパネルが表示位置を設定済みの時だけ操作できる
  const bool linked  = view.view_size[0] > 1 && view.canvas_size == Vec2((float)doc->width(), (float)doc->height());
  const float footer = ImGui::GetFrameHeightWithSpacing() * 2;
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 area(std::max(avail.x, 1.0f), std::max(avail.y - footer, 32.0f));
  const float fit = std::min(area.x / doc->width(), area.y / doc->height());
  const ImVec2 size(doc->width() * fit, doc->height() * fit);
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  const ImVec2 min(cursor.x + (area.x - size.x) * 0.5f, cursor.y + (area.y - size.y) * 0.5f);
  const ImVec2 max(min.x + size.x, min.y + size.y);

  // クリック/ドラッグした所をキャンバスの中央に表示する
  ImGui::InvisibleButton("##thumbnail", area);
  const Vec2 center(view.view_pos[0] + view.view_size[0] * 0.5f, view.view_pos[1] + view.view_size[1] * 0.5f);
  if(linked && ImGui::IsItemActive()) {
    const ImVec2 m = ImGui::GetIO().MousePos;
    const Vec2 s   = view.to_screen(Vec2((m.x - min.x) / fit, (m.y - min.y) / fit));
    view.move(Vec2(center[0] - s[0], center[1] - s[1]));
  }

  auto* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(min, max, IM_COL32_WHITE); // 透明な所は白い紙に見せる
  if(tex_->img) dl->AddImage((ImTextureID)(intptr_t)tex_->gl.get_id(), min, max);
  dl->AddRect(min, max, ImGui::GetColorU32(ImGuiCol_Border));
  if(linked) { // 表示中の範囲
    const float x0 = view.view_pos[0], y0 = view.view_pos[1], x1 = x0 + view.view_size[0], y1 = y0 + view.view_size[1];
    ImVec2 q[4] = {ImVec2(x0, y0), ImVec2(x1, y0), ImVec2(x1, y1), ImVec2(x0, y1)};
    for(ImVec2& p : q) {
      const Vec2 c = view.to_canvas(Vec2(p.x, p.y));
      p            = ImVec2(min.x + c[0] * fit, min.y + c[1] * fit);
    }
    dl->PushClipRect(cursor, ImVec2(cursor.x + area.x, cursor.y + area.y), true);
    dl->AddPolyline(q, 4, IM_COL32(255, 60, 60, 255), ImDrawFlags_Closed, 1.5f);
    dl->PopClipRect();
  }

  ImGui::BeginDisabled(!linked);
  ImGui::SetNextItemWidth(-FLT_MIN);
  float percent = view.zoom * 100.0f;
  if(ImGui::SliderFloat("##zoom", &percent, CanvasView::kMinZoom * 100, CanvasView::kMaxZoom * 100, "%.1f%%", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp)) view.zoom_at(center, percent / 100.0f / view.zoom);
  ImGui::SetItemTooltip("拡大率");
  const float button = std::min(ImGui::GetFrameHeight() * 1.3f, (area.x - ImGui::GetStyle().ItemSpacing.x * 5) / 6);
  bool first         = true;
  const auto tool    = [&](const char* icon, const char* tip) {
    if(!std::exchange(first, false)) ImGui::SameLine();
    const bool clicked = ImGui::Button(icon, ImVec2(button, 0));
    ImGui::SetItemTooltip("%s", tip);
    return clicked;
  };
  constexpr float kStep = 3.14159265f / 12; // 15度
  if(tool(ICON_FA_MAGNIFYING_GLASS_MINUS "##out", "縮小")) view.zoom_at(center, 1.0f / 1.25f);
  if(tool(ICON_FA_MAGNIFYING_GLASS_PLUS "##in", "拡大")) view.zoom_at(center, 1.25f);
  if(tool(ICON_FA_EXPAND "##fit", "全体表示")) view.fit();
  if(tool(ICON_FA_ROTATE_LEFT "##ccw", "左回転")) view.rotate_at(center, -kStep);
  if(tool(ICON_FA_ROTATE_RIGHT "##cw", "右回転")) view.rotate_at(center, kStep);
  if(tool(ICON_FA_RIGHT_LEFT "##flip", "左右反転")) view.flip_horizontal();
  ImGui::EndDisabled();
  ImGui::End();
}

} // namespace mu
