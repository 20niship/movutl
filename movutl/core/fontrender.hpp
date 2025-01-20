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
class ImageRGBA;

namespace detail {
struct FontRenderManager {
  MOVUTL_DECLARE_SINGLETON(FontRenderManager)
  FontRenderManager() { init(); }
  ~FontRenderManager() { shutdown(); }

  bool initialized = false;
  FT_Library library = nullptr;

  struct FontFace {
  private:
    void draw_bitmap(ImageRGBA* img, int x, int y);
    void drawString(const char text[]);
    int fontsize_ = 0;

  public:
    FT_Face face = nullptr;
    FT_GlyphSlot slot = nullptr; // グリフへのショートカット
    std::string path;
    std::u32string u32str;
    FontFace() = default;
    FontFace(const std::string& path, int width = 16);
    void render_text(const char* text, int space_x, int space_y, ImageRGBA* img);
    Vec2d get_size(const char* text);
    void set_fontsize(int size);
    ~FontFace();
  };
  std::unordered_map<std::string, FontFace> font_faces;

  void init();
  void shutdown();
  static bool renderText(ImageRGBA* img, const char* text, int size, int sace_x, int space_y, const char* font_name);
};
} // namespace detail
} // namespace mu
