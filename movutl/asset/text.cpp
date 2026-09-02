#include <cmath>
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

bool TextEntt::render(Composition* cmp, Image* target, int frame) {
  if(text.empty()) return true;
  re_render_image();
  if(!img_ || img_->empty() || !cmp || !target) return false;
  render_filters(cmp, img_.get(), frame);
  Vec2d center(pos_[0] + text_offset_[0] + img_->width * scale_x_ / 2.0, pos_[1] + text_offset_[1] + img_->height * scale_y_ / 2.0);
  float rot_deg = rot_ * 180.0f / (float)M_PI; // rot_はradians=trueプロパティ、copyto()はdegreesを期待する
  img_->copyto(target, center, (scale_x_ + scale_y_) / 2.0f, rot_deg, alpha_ / 255.0f);
  return true;
}

void TextEntt::re_render_image() {
  bool need = last_text_ != text || last_font_ != font || last_color_ != color_ || last_border_color_ != border_color_ || last_border_width_ != border_width_;
  if(!need) return;
  last_text_         = text;
  last_font_         = font;
  last_color_        = color_;
  last_border_color_ = border_color_;
  last_border_width_ = border_width_;
  if(font.empty()) {
    LOG_F(ERROR, "TextEntt: no font available");
    return;
  }
  if(!img_) img_ = cutil::make_ref<Image>();
  using namespace detail;
  FontRenderManager::renderText(img_.get(), text.c_str(), 16, 0, 0, font.c_str(), color_);
  text_offset_ = Vec2(0, 0);
  if(border_width_ > 0) {
    // outline()は既存の不透明部分の外側にしか描けないので、先にキャンバスへ枠線分の余白を足す
    int pad     = border_width_;
    auto padded = cutil::make_ref<Image>();
    padded->resize((int)img_->width + pad * 2, (int)img_->height + pad * 2);
    padded->has_alpha = true;
    padded->fill(0);
    img_->copyto(padded.get(), Vec2d(pad, pad));
    img_         = padded;
    text_offset_ = Vec2(-(float)pad, -(float)pad);
    img_->outline(border_color_, pad);
  }
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
