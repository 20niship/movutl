#include <codecvt>
#include <iostream>
#include <locale>
#include <math.h>
#include <stdio.h>
#include <string>

#include <movutl/core/assert.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/fontrender.hpp>
#include <movutl/core/imagebase.hpp>
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
  auto library = FontRenderManager::Get()->library;
  MU_ASSERT(library);
  auto error = FT_New_Face(library, path.c_str(), 0, &face);
  if(error == FT_Err_Unknown_File_Format) {
    LOG_F(ERROR, "Font format is unsupported");
  } else if(error) {
    LOG_F(ERROR, "Failed to open font file: %d %s", error, path.c_str());
  }
  set_fontsize(width);
}

FontRenderManager::FontFace::~FontFace() {
  /// FT_Done_Face(face);
}

Vec2d FontRenderManager::FontFace::get_size(const char* text) {
  MU_ASSERT(face);
  MU_ASSERT(slot);
  std::u32string u32str = std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t>().from_bytes(text);

  int curPosX = 0;
  int curPosY = 60;    // 現在のカーソル位置
  int last_height = 0; // 最後に文字を書いたときの文字の大きさ
  slot = face->glyph;  // グリフへのショートカット

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
  MU_ASSERT(face);
  /*auto error = FT_Set_Char_Size(face, 0, size * size * 2, 300, 300);*/
  auto error = FT_Set_Pixel_Sizes(face, 0, 48);
  if(error) {
    LOG_F(ERROR, "Failed to set font size: %d", error);
  }
  fontsize_ = size;
  slot = face->glyph; // グリフへのショートカット
}

void FontRenderManager::FontFace::render_text(const char* text, int space_x, int space_y, ImageRGBA* img) {
  MU_ASSERT(face);
  MU_ASSERT(slot);
  MU_ASSERT(img);
  auto size = get_size(text);
  img->resize(size[0], size[1]);

  std::u32string u32str = std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t>().from_bytes(text);

  int curPosX = 0;
  int curPosY = 60;    // 現在のカーソル位置
  int last_height = 0; // 最後に文字を書いたときの文字の大きさ

  for(int n = 0; n < u32str.size(); n++) {
    if(u32str[n] == '\n') {
      curPosX = 0;
      curPosY += last_height + 20;
    } else {
      if(FT_Load_Char(face, u32str[n], FT_LOAD_RENDER)) continue; // 一文字レンダリング
      // int yMax = face->bbox.yMax;
      // int yMin = face->bbox.yMin;
      // int baseline = bitmap->rows * yMax / (yMax - yMin);
      draw_bitmap(img, curPosX, curPosY - slot->bitmap_top); // imageにslot->bitmapの中身をコピーする
    }
    last_height = (slot->bitmap).rows;

    curPosX += slot->advance.x >> 6;
    curPosY += slot->advance.y >> 6;
  }
};

// 生成された位置も自分の画像データをimageにコピーする
void FontRenderManager::FontFace::draw_bitmap(ImageRGBA* img, int x, int y) {
  int i, j, p, q;
  const int x_max = x + (slot->bitmap).width;
  const int y_max = y + (slot->bitmap).rows;

  for(j = y, q = 0; j < y_max; j++, q++) {
    Vec4b* image = img->data();
    for(i = x, p = 0; i < x_max; i++, p++) {
      if(i < 0 || j < 0 || i >= img->width || j >= img->height) continue;
      char color = (slot->bitmap).buffer[q * (slot->bitmap).width + p];
      Vec4b* pixel = &image[j * img->width + i];
      (*pixel)[0] = color | (*pixel)[0];
      (*pixel)[1] = color | (*pixel)[1];
      (*pixel)[2] = color | (*pixel)[2];
      (*pixel)[3] = color | (*pixel)[3];
    }
  }
}

bool FontRenderManager::renderText(ImageRGBA* img, const char* text, int size, int sace_x, int space_y, const char* font_name) {
  auto manager = FontRenderManager::Get();
  if(!manager->initialized) {
    LOG_F(ERROR, "FontRenderManager is not initialized");
    return false;
  }

  if(!img) return false;
  for(auto& [name, font_face] : manager->font_faces) {
    if(name == font_name) {
      font_face.set_fontsize(size);
      font_face.render_text(text, sace_x, space_y, img);
      return true;
    }
  }
  {
    manager->font_faces[font_name] = FontFace(font_name, size);
    manager->font_faces[font_name].render_text(text, sace_x, space_y, img);
    return true;
  }
  return false;
}

FontRenderManager* FontRenderManager::singleton_ = nullptr;

} // namespace detail
} // namespace mu
