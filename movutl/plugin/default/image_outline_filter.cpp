#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_outline_filter.hpp>
#include <opencv2/opencv.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- 縁取り ----------------
bool fn_proc_outline(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Outline::fn_proc");

  Vec4b color = cutil::get_or<Vec4b>(p, "color", Vec4b(0, 0, 0, 255));
  int width   = (int)cutil::get_or<float>(p, "width", 3.0f);
  fpip->img->outline(color, width);
  return true;
}
bool fn_init_outline(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("color", 0, cutil::prop_info_of<Vec4b>()));
  props->fields.back().set_label("色");
  defaults->set<Vec4b>("color", Vec4b(0, 0, 0, 255));

  props->fields.push_back(cutil::PropInfo::Field("width", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("太さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("width", 3.0f);
  return true;
}
FilterPluginTable f_outline = {
  GUID(0x0000d), FilterDefault, cutil::Str("縁取り"), cutil::Str("縁取り"), 0, "0", nullptr, nullptr, fn_init_outline, nullptr, fn_proc_outline, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- クリッピング&リサイズ ----------------
bool fn_proc_clipping(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Clipping::fn_proc");

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  int left         = (int)cutil::get_or<float>(p, "left", 0.0f);
  int top          = (int)cutil::get_or<float>(p, "top", 0.0f);
  int right        = (int)cutil::get_or<float>(p, "right", 0.0f);
  int bottom       = (int)cutil::get_or<float>(p, "bottom", 0.0f);
  bool resize_back = cutil::get_or<bool>(p, "resize", true);
  if(left == 0 && top == 0 && right == 0 && bottom == 0) return true;

  cv::Rect roi(left, top, w - left - right, h - top - bottom);
  roi &= cv::Rect(0, 0, w, h);
  if(roi.width <= 0 || roi.height <= 0) return true;

  cv::Mat src;
  img->to_cv_img(&src);
  cv::Mat cropped = src(roi).clone();

  if(resize_back) {
    cv::Mat resized;
    cv::resize(cropped, resized, cv::Size(w, h));
    img->set_cv_img(&resized);
  } else {
    img->set_cv_img(&cropped);
  }
  return true;
}
bool fn_init_clipping(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  const char* names[4]  = {"left", "top", "right", "bottom"};
  const char* labels[4] = {"左", "上", "右", "下"};
  for(int i = 0; i < 4; i++) {
    props->fields.push_back(cutil::PropInfo::Field(names[i], 0, cutil::prop_info_of<float>()));
    props->fields.back().set_label(labels[i]);
    props->fields.back().min_value  = 0.0f;
    props->fields.back().max_value  = 10000.0f;
    props->fields.back().drag_speed = 1.0f;
    defaults->set<float>(names[i], 0.0f);
  }
  props->fields.push_back(cutil::PropInfo::Field("resize", 0, cutil::prop_info_of<bool>()));
  props->fields.back().set_label("クリップ後に元サイズへ拡大");
  defaults->set<bool>("resize", true);
  return true;
}
FilterPluginTable f_clipping = {
  GUID(0x0000e), FilterDefault, cutil::Str("クリッピング&リサイズ"), cutil::Str("クリッピング&リサイズ"), 0, "0", nullptr, nullptr, fn_init_clipping, nullptr, fn_proc_clipping, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
