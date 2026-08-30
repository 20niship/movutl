#include <codecvt>
#include <iostream>
#include <locale>
#include <math.h>
#include <stdio.h>
#include <string>

#include <movutl/asset/image.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/fontrender.hpp>
#include <movutl/core/logger.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H
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

FontRenderManager::FontFace::FontFace(const std::string& path, int width) {
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
    return;
  } else if(error) {
    LOG_F(ERROR, "Failed to open font file: %d %s", error, path.c_str());
    return;
  }
  set_fontsize(width);
}

FontRenderManager::FontFace::~FontFace() {
  /// FT_Done_Face(face);
}

Vec2d FontRenderManager::FontFace::get_size(const char* text) {
  if(face == nullptr || slot == nullptr) return Vec2d(0, 0);
  std::u32string u32str = std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t>().from_bytes(text);

  int curPosX     = 0;
  int curPosY     = 60;          // 現在のカーソル位置
  int last_height = 0;           // 最後に文字を書いたときの文字の大きさ
  slot            = face->glyph; // グリフへのショートカット

  for(int n = 0; n < u32str.size(); n++) {
    if(u32str[n] == '\n') {
      curPosX = 0;
      curPosY += last_height + 20;
    } else {
      if(FT_Load_Char(face, u32str[n], FT_LOAD_RENDER)) continue; // 一文字レンダリング
    }
    last_height = (slot->bitmap).rows;

    curPosX += slot->advance.x >> 6;
    curPosY += slot->advance.y >> 6;
  }
  return Vec2d(curPosX, curPosY);
}

void FontRenderManager::FontFace::set_fontsize(int size) {
  if(fontsize_ == size) return;
  if(face == nullptr) {
    LOG_F(ERROR, "set_fontsize: font face is not loaded: %s", path.c_str());
    return;
  }
  auto error = FT_Set_Pixel_Sizes(face, 0, 48);
  if(error) {
    LOG_F(ERROR, "Failed to set font size: %d", error);
  }
  fontsize_ = size;
  slot      = face->glyph; // グリフへのショートカット
}

void FontRenderManager::FontFace::render_text(const char* text, int space_x, int space_y, Image* img, const Vec4b& color) {
  if(face == nullptr || slot == nullptr || img == nullptr) return;
  auto size = get_size(text);
  img->resize(size[0], size[1]);
  img->fill(0);

  std::u32string u32str = std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t>().from_bytes(text);

  int curPosX     = 0;
  int curPosY     = 60; // 現在のカーソル位置
  int last_height = 0;  // 最後に文字を書いたときの文字の大きさ

  for(int n = 0; n < u32str.size(); n++) {
    if(u32str[n] == '\n') {
      curPosX = 0;
      curPosY += last_height + 20;
    } else {
      if(FT_Load_Char(face, u32str[n], FT_LOAD_RENDER)) continue; // 一文字レンダリング
      // int yMax = face->bbox.yMax;
      // int yMin = face->bbox.yMin;
      // int baseline = bitmap->rows * yMax / (yMax - yMin);
      draw_bitmap(img, curPosX, curPosY - slot->bitmap_top, color); // imageにslot->bitmapの中身をコピーする
    }
    last_height = (slot->bitmap).rows;

    curPosX += slot->advance.x >> 6;
    curPosY += slot->advance.y >> 6;
    curPosX += space_x;
  }
};

// 生成された位置も自分の画像データをimageにコピーする
void FontRenderManager::FontFace::draw_bitmap(Image* img, int x, int y, const Vec4b& color) {
  int i, j, p, q;
  const int x_max = x + (slot->bitmap).width;
  const int y_max = y + (slot->bitmap).rows;

  for(j = y, q = 0; j < y_max; j++, q++) {
    Vec4b* image = img->data();
    for(i = x, p = 0; i < x_max; i++, p++) {
      if(i < 0 || j < 0 || i >= img->width || j >= img->height) continue;
      unsigned char coverage = (slot->bitmap).buffer[q * (slot->bitmap).width + p];
      if(coverage == 0) continue;
      Vec4b* pixel = &image[j * img->width + i];
      // グリフの縁が重なる箇所はアルファが大きい方を残す(小さい方で上書きしない)
      if(coverage > (*pixel)[3]) {
        (*pixel)[0] = color[0];
        (*pixel)[1] = color[1];
        (*pixel)[2] = color[2];
        (*pixel)[3] = coverage;
      }
    }
  }
}

bool FontRenderManager::renderText(Image* img, const char* text, int size, int sace_x, int space_y, const char* font_name, const Vec4b& color) {
  auto manager = FontRenderManager::Get();
  if(!manager->initialized) {
    LOG_F(ERROR, "FontRenderManager is not initialized");
    return false;
  }

  if(!img) return false;
  for(auto& [name, font_face] : manager->font_faces) {
    if(name == font_name) {
      if(font_face.face == nullptr) return false;
      font_face.set_fontsize(size);
      font_face.render_text(text, sace_x, space_y, img, color);
      return true;
    }
  }
  {
    manager->font_faces[font_name] = FontFace(font_name, size);
    auto& font_face                = manager->font_faces[font_name];
    if(font_face.face == nullptr) {
      LOG_F(ERROR, "Failed to load font: %s", font_name);
      return false;
    }
    font_face.render_text(text, sace_x, space_y, img, color);
    return true;
  }
  return false;
}

FontRenderManager* FontRenderManager::singleton_ = nullptr;

} // namespace detail
} // namespace mu
