#include <algorithm>
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
  composite(*img_, target, align_origin_offset());
  return true;
}

Vec2 TextEntt::align_origin_offset() const {
  if(!img_) return Vec2(0, 0);
  const int a    = std::clamp((int)align_, 0, 8);
  const float bw = (float)img_->width - 2 * pad_, bh = (float)img_->height - 2 * pad_;
  return Vec2((a % 3 - 1) * bw / 2.0f, (a / 3 - 1) * bh / 2.0f);
}

void TextEntt::re_render_image() {
  std::string key = text + '\x1f' + font;
  for(int v : {(int)font_size_, (int)spacing_x_, (int)spacing_y_, (int)align_, (int)deco_, (int)bold_, (int)italic_, (int)monospace_, (int)color_[0], (int)color_[1], (int)color_[2], (int)color_[3], (int)deco_color_[0], (int)deco_color_[1], (int)deco_color_[2], (int)deco_color_[3]})
    key += '\x1f' + std::to_string(v);
  if(!last_key_.empty() && key == last_key_) return;
  if(font.empty()) {
    LOG_F(ERROR, "TextEntt: no font available");
    return;
  }
  if(!img_) img_ = cutil::make_ref<Image>();
  using namespace detail;
  TextStyle st;
  st.size       = font_size_;
  st.bold       = bold_;
  st.italic     = italic_;
  st.spacing_x  = spacing_x_;
  st.spacing_y  = spacing_y_;
  st.monospace  = monospace_;
  st.line_align = std::clamp((int)align_, 0, 8) % 3; // 複数行の行内揃えは揃えの左/中央/右と同じ
  if(!FontRenderManager::renderText(img_.get(), text.c_str(), font.c_str(), st, color_)) return;
  last_key_ = key;
  pad_      = 0;

  // 装飾のために四辺へ同じ余白pad_を足した画像にする(揃えの基準になる文字ブロックの位置が余白で偏らないように対称にする)
  int outline_w = 0, shadow_d = 0;
  float shadow_alpha = 1.0f;
  switch(deco_) {
    case TextDeco_Shadow: shadow_d = std::max(1, font_size_ / 16); break;
    case TextDeco_ShadowLight:
      shadow_d     = std::max(1, font_size_ / 16);
      shadow_alpha = 0.5f;
      break;
    case TextDeco_Outline: outline_w = std::max(1, font_size_ / 12); break;
    case TextDeco_OutlineThin: outline_w = std::max(1, font_size_ / 24); break;
    default: break;
  }
  const int pad = std::max(outline_w, shadow_d);
  if(pad == 0) return;
  auto padded = cutil::make_ref<Image>();
  padded->resize((int)img_->width + pad * 2, (int)img_->height + pad * 2);
  padded->has_alpha = true;
  padded->fill(0);
  if(shadow_d > 0) {
    Image shadow(img_->width, img_->height);
    shadow.has_alpha = true;
    for(size_t i = 0; i < shadow.size(); i++) shadow[i] = Vec4b(deco_color_[0], deco_color_[1], deco_color_[2], (*img_)[i][3]);
    shadow.copyto(padded.get(), Vec2d(pad + shadow_d, pad + shadow_d), shadow_alpha);
  }
  if(outline_w > 0) {
    // outline()は既存の不透明部分の外側にしか描けないので、余白を足したキャンバスへ文字を置いてから縁取る
    img_->copyto(padded.get(), Vec2d(pad, pad));
    padded->outline(deco_color_, outline_w);
  } else {
    img_->copyto(padded.get(), Vec2d(pad, pad));
  }
  img_ = padded;
  pad_ = pad;
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
