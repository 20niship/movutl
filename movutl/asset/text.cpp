#include <movutl/asset/project.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/fontrender.hpp>
#include <opencv2/opencv.hpp>

namespace mu {
bool TextEntt::render(Composition* cmp) {
  re_render_image();
  if(!img_) return false;
  img_->copyto(cmp->frame_final.get(), Vec2d(pos_.xy()));
  return false;
}

void TextEntt::re_render_image() {
  if(last_text_ == text && last_font_ == font) return;
  last_text_ = text;
  last_font_ = font;
  if(!img_) img_ = std::make_shared<ImageRGBA>();
  using namespace detail;
  bool reslt = FontRenderManager::renderText(img_.get(), text.c_str(), 16, 0, 0, font.c_str());

  if(!img_->empty()) {
    static cv::Mat mat;
    img_->to_cv_img(&mat);
    cv::imshow("text", mat);
    cv::waitKey(1);
  }
}

Ref<TextEntt> TextEntt::Create(const char* text, const char* font) {
  auto ent = Ref<TextEntt>(new TextEntt());
  ent->name = "text";
  ent->text = text;
  auto all_fonts = get_available_fonts();
  if(font && std::string(font).size() > 0) {
    ent->font = font;
  } else if(all_fonts.size() > 0) {
    for(auto f : all_fonts) {
      if(f.ends_with(".ttf")) {
        ent->font = f;
        break;
      }
    }
  }
  return ent;
}

} // namespace mu
