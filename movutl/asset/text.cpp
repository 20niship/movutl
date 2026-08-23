#include <movutl/asset/project.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/fontrender.hpp>
#include <movutl/core/logger.hpp>

namespace mu {

static std::string find_default_font() {
  /// 1. バンドルフォント (assets/fonts/) を優先
  const std::string bundled = fs_get_font_path() + "/Meiryo.ttf";
  if(fs_exists(bundled)) return bundled;

  /// 2. システムフォントから選択(明朝を優先、なければ最初のもの)
  const auto all_fonts = get_available_fonts();
  if(all_fonts.empty()) return "";
  for(const auto& f : all_fonts) {
    if(f.find("明朝") != std::string::npos) return f;
  }
  return all_fonts.front();
}

bool TextEntt::render(Composition* cmp) {
  if(text.empty()) return true;
  re_render_image();
  if(!img_ || img_->empty() || !cmp || !cmp->frame_final) return false;
  img_->copyto(cmp->frame_final.get(), Vec2d(pos_.xy()));
  return true;
}

void TextEntt::re_render_image() {
  if(last_text_ == text && last_font_ == font) return;
  last_text_ = text;
  last_font_ = font;
  if(font.empty()) {
    LOG_F(ERROR, "TextEntt: no font available");
    return;
  }
  if(!img_) img_ = cutil::make_ref<Image>();
  using namespace detail;
  FontRenderManager::renderText(img_.get(), text.c_str(), 16, 0, 0, font.c_str());
}

Ref<TextEntt> TextEntt::Create(const char* text, const char* font) {
  auto ent  = Ref<TextEntt>(new TextEntt());
  ent->name = "text";
  ent->text = text;

  if(font != nullptr && fs_exists(font)) {
    ent->font = font;
  } else {
    ent->font = find_default_font();
  }
  return ent;
}

} // namespace mu
