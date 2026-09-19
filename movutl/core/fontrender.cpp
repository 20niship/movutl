#include <codecvt>
#include <iostream>
#include <locale>
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <string>
#include <vector>

#include <movutl/asset/image.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/fontrender.hpp>
#include <movutl/core/logger.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H
#include FT_SYNTHESIS_H
#include <opencv2/opencv.hpp>

#define WIDTH 500
#define HEIGHT 300


namespace mu {

namespace detail {

void FontRenderManager::init() {
  if(initialized) return;
  auto error = FT_Init_FreeType(&library);
  if(error) {
    LOG_F(ERROR, "Failed to initialize FreeType library: %d", error);
    return;
  }
  initialized = true;
}

void FontRenderManager::shutdown() {
  if(!initialized) return;
  for(auto& [name, font_face] : font_faces) {
    FT_Done_Face(font_face.face);
  }
  FT_Done_FreeType(library);
  initialized = false;
}

FontRenderManager::FontFace::FontFace(const std::string& path) {
  this->path = path;
  if(!fs_exists(path)) {
    LOG_F(ERROR, "Font file not found: %s", path.c_str());
    return;
  }
  auto library = FontRenderManager::Get()->library;
  MU_ASSERT(library);
  auto error = FT_New_Face(library, path.c_str(), 0, &face);
  if(error == FT_Err_Unknown_File_Format) {
    LOG_F(ERROR, "Font format is unsupported: %s", path.c_str());
    face = nullptr;
  } else if(error) {
    LOG_F(ERROR, "Failed to open font file: %d %s", error, path.c_str());
    face = nullptr;
  }
}

FontRenderManager::FontFace::~FontFace() {
  /// FT_Done_Face(face);
}

namespace {
// グリフを(必要なら太字・斜体を合成して)ラスタライズする。成功でtrue
bool load_glyph(FT_Face face, char32_t ch, const TextStyle& st) {
  if(FT_Load_Char(face, ch, FT_LOAD_DEFAULT | FT_LOAD_NO_BITMAP)) return false;
  FT_GlyphSlot slot = face->glyph;
  if(slot->format == FT_GLYPH_FORMAT_OUTLINE) {
    if(st.bold) FT_Outline_Embolden(&slot->outline, std::max(1, st.size / 24) * 64);
    if(st.italic) FT_GlyphSlot_Oblique(slot);
  }
  return FT_Render_Glyph(slot, FT_RENDER_MODE_NORMAL) == 0;
}

int glyph_advance(FT_Face face, char32_t ch, const TextStyle& st) {
  if(st.monospace) return ch < 0x80 ? st.size / 2 : st.size;
  return (int)(face->glyph->advance.x >> 6) + (st.bold ? std::max(1, st.size / 24) : 0);
}

// グリフの縁が重なる箇所はアルファが大きい方を残す(小さい方で上書きしない)
void draw_bitmap(Image* img, FT_GlyphSlot slot, int x, int y, const Vec4b& color) {
  const auto& bm = slot->bitmap;
  for(unsigned q = 0; q < bm.rows; q++) {
    for(unsigned p = 0; p < bm.width; p++) {
      const int i = x + (int)p, j = y + (int)q;
      if(i < 0 || j < 0 || i >= (int)img->width || j >= (int)img->height) continue;
      const unsigned char coverage = bm.buffer[q * bm.pitch + p];
      if(coverage == 0) continue;
      Vec4b& px = img->data()[j * img->width + i];
      if(coverage > px[3]) px = Vec4b(color[0], color[1], color[2], coverage);
    }
  }
}
} // namespace

void FontRenderManager::FontFace::render_text(const char* text, const TextStyle& st, Image* img, const Vec4b& color) {
  if(face == nullptr || img == nullptr) return;
  FT_Set_Pixel_Sizes(face, 0, std::max(1, st.size));
  const int ascender = (int)(face->size->metrics.ascender >> 6);
  const int line_h   = std::max(1, (int)(face->size->metrics.height >> 6));

  std::u32string u32 = std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t>().from_bytes(text);
  std::vector<std::u32string> lines(1);
  for(char32_t ch : u32) {
    if(ch == U'\r') continue;
    if(ch == U'\n') lines.emplace_back();
    else lines.back() += ch;
  }

  // 1パス目: 各行の幅
  std::vector<int> widths;
  int max_w = 1;
  for(auto& ln : lines) {
    int w = 0;
    for(char32_t ch : ln) {
      if(!load_glyph(face, ch, st)) continue;
      w += glyph_advance(face, ch, st) + st.spacing_x;
    }
    w = std::max(0, w - (ln.empty() ? 0 : st.spacing_x));
    widths.push_back(w);
    max_w = std::max(max_w, w);
  }
  const int slack = st.italic ? st.size / 4 : 0; // 斜体の右端はadvanceからはみ出す
  const int W     = max_w + slack;
  const int H     = (int)lines.size() * line_h + ((int)lines.size() - 1) * st.spacing_y;
  img->resize(W, H);
  img->has_alpha = true;
  img->fill(0);

  // 2パス目: 描画
  for(size_t i = 0; i < lines.size(); i++) {
    int x = st.line_align == 1 ? (max_w - widths[i]) / 2 : st.line_align == 2 ? max_w - widths[i] : 0;
    const int baseline = (int)i * (line_h + st.spacing_y) + ascender;
    for(char32_t ch : lines[i]) {
      if(!load_glyph(face, ch, st)) continue;
      const int adv = glyph_advance(face, ch, st);
      // 等間隔のときはグリフをセルの中央に置く
      const int cell_off = st.monospace ? std::max(0, (adv - (int)(face->glyph->advance.x >> 6)) / 2) : 0;
      draw_bitmap(img, face->glyph, x + cell_off + face->glyph->bitmap_left, baseline - face->glyph->bitmap_top, color);
      x += adv + st.spacing_x;
    }
  }
}

bool FontRenderManager::renderText(Image* img, const char* text, const char* font_name, const TextStyle& style, const Vec4b& color) {
  auto manager = FontRenderManager::Get();
  if(!manager->initialized) {
    LOG_F(ERROR, "FontRenderManager is not initialized");
    return false;
  }
  if(!img) return false;
  auto it = manager->font_faces.find(font_name);
  if(it == manager->font_faces.end()) it = manager->font_faces.emplace(font_name, FontFace(font_name)).first;
  if(it->second.face == nullptr) {
    LOG_F(ERROR, "Failed to load font: %s", font_name);
    return false;
  }
  it->second.render_text(text, style, img, color);
  return true;
}

FontRenderManager* FontRenderManager::singleton_ = nullptr;

} // namespace detail
} // namespace mu
