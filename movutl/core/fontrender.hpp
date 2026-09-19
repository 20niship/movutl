#pragma once
#include <memory>
#include <movutl/core/defines.hpp>
#include <movutl/core/vector.hpp>
#include <string>
#include <unordered_map>

struct FT_LibraryRec_;
struct FT_FaceRec_;
struct FT_GlyphSlotRec_;
typedef struct FT_LibraryRec_* FT_Library;
typedef struct FT_FaceRec_* FT_Face;
typedef struct FT_GlyphSlotRec_* FT_GlyphSlot;

namespace mu {
class Image;

namespace detail {

// テキスト描画のスタイル。sizeはピクセル単位のフォントサイズ
struct TextStyle {
  int size        = 34;
  bool bold       = false;
  bool italic     = false;
  int spacing_x   = 0;     // 字間(px)
  int spacing_y   = 0;     // 行間(px)
  bool monospace  = false; // 等間隔(半角=size/2, 全角=size)
  int line_align  = 0;     // 複数行のときの行内揃え。0:左 1:中央 2:右
};

struct FontRenderManager {
  MOVUTL_DECLARE_SINGLETON(FontRenderManager)
  FontRenderManager() { init(); }
  ~FontRenderManager() { shutdown(); }

  bool initialized   = false;
  FT_Library library = nullptr;

  struct FontFace {
  public:
    FT_Face face = nullptr;
    std::string path;
    FontFace() = default;
    explicit FontFace(const std::string& path);
    ~FontFace();
    // textをstyleに従って改行(\n)ごとに組み、imgを必要サイズにリサイズして描く
    void render_text(const char* text, const TextStyle& style, Image* img, const Vec4b& color);
  };
  std::unordered_map<std::string, FontFace> font_faces;

  void init();
  void shutdown();
  static bool renderText(Image* img, const char* text, const char* font_name, const TextStyle& style = TextStyle(), const Vec4b& color = Vec4b(255, 255, 255, 255));
};
} // namespace detail
} // namespace mu
