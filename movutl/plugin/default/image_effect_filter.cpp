#include <algorithm>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_effect_filter.hpp>
#include <opencv2/opencv.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- ノイズ除去(メディアンフィルタ、alphaは維持) ----------------
bool fn_proc_denoise(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Denoise::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 30.0f);
  if(strength <= 0.0f) return true;
  int ksize = std::clamp(((int)(strength / 10)) * 2 + 1, 3, 15);

  cv::Mat src;
  fpip->img->to_cv_img(&src);
  std::vector<cv::Mat> ch(4);
  cv::split(src, ch);
  cv::Mat bgr;
  cv::merge(std::vector<cv::Mat>{ch[0], ch[1], ch[2]}, bgr);
  cv::Mat bgr_out;
  cv::medianBlur(bgr, bgr_out, ksize);
  std::vector<cv::Mat> out_ch;
  cv::split(bgr_out, out_ch);
  out_ch.push_back(ch[3]);
  cv::Mat dst;
  cv::merge(out_ch, dst);
  fpip->img->set_cv_img(&dst);
  return true;
}
bool fn_init_denoise(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 30.0f);
  return true;
}
FilterPluginTable f_denoise = {
  GUID(0x00012), FilterDefault, cutil::Str("ノイズ除去"), cutil::Str("ノイズ除去"), 0, "0", nullptr, nullptr, fn_init_denoise, nullptr, fn_proc_denoise, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- シャープ(アンシャープマスク) ----------------
bool fn_proc_sharpen(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Sharpen::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 50.0f) / 100.0f;
  if(strength <= 0.0f) return true;

  cv::Mat src;
  fpip->img->to_cv_img(&src);
  cv::Mat blurred;
  cv::GaussianBlur(src, blurred, cv::Size(0, 0), 3.0);
  cv::Mat sharpened;
  cv::addWeighted(src, 1.0 + strength, blurred, -strength, 0, sharpened);
  fpip->img->set_cv_img(&sharpened);
  return true;
}
bool fn_init_sharpen(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 500.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 50.0f);
  return true;
}
FilterPluginTable f_sharpen = {
  GUID(0x00013), FilterDefault, cutil::Str("シャープ"), cutil::Str("シャープ"), 0, "0", nullptr, nullptr, fn_init_sharpen, nullptr, fn_proc_sharpen, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- エッジ抽出 ----------------
bool fn_proc_edge_detect(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("EdgeDetect::fn_proc");

  float threshold = cutil::get_or<float>(p, "threshold", 100.0f);

  cv::Mat src;
  fpip->img->to_cv_img(&src);
  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
  cv::Mat edges;
  cv::Canny(gray, edges, threshold * 0.5, threshold);
  cv::Mat out;
  cv::cvtColor(edges, out, cv::COLOR_GRAY2BGRA);
  fpip->img->set_cv_img(&out);
  return true;
}
bool fn_init_edge_detect(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("threshold", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("しきい値");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 500.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("threshold", 100.0f);
  return true;
}
FilterPluginTable f_edge_detect = {
  GUID(0x00014), FilterDefault, cutil::Str("エッジ抽出"), cutil::Str("エッジ抽出"), 0, "0", nullptr, nullptr, fn_init_edge_detect, nullptr, fn_proc_edge_detect, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- モザイク ----------------
bool fn_proc_mosaic(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Mosaic::fn_proc");

  int block = (int)cutil::get_or<float>(p, "block_size", 10.0f);
  if(block <= 1) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  cv::Mat src;
  img->to_cv_img(&src);
  cv::Mat small;
  cv::resize(src, small, cv::Size(std::max(1, w / block), std::max(1, h / block)), 0, 0, cv::INTER_LINEAR);
  cv::Mat mosaic;
  cv::resize(small, mosaic, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
  img->set_cv_img(&mosaic);
  return true;
}
bool fn_init_mosaic(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("block_size", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("ブロックサイズ");
  props->fields.back().min_value  = 1.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("block_size", 10.0f);
  return true;
}
FilterPluginTable f_mosaic = {
  GUID(0x00015), FilterDefault, cutil::Str("モザイク"), cutil::Str("モザイク"), 0, "0", nullptr, nullptr, fn_init_mosaic, nullptr, fn_proc_mosaic, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- リサイズ ----------------
bool fn_proc_resize(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Resize::fn_proc");

  float scale = cutil::get_or<float>(p, "scale", 100.0f) / 100.0f;
  if(scale <= 0.0f || scale == 1.0f) return true;

  Image* img = fpip->img;
  int nw = std::max(1, (int)(img->width * scale));
  int nh = std::max(1, (int)(img->height * scale));
  cv::Mat src;
  img->to_cv_img(&src);
  cv::Mat resized;
  cv::resize(src, resized, cv::Size(nw, nh));
  img->set_cv_img(&resized);
  return true;
}
bool fn_init_resize(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("scale", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("拡大率");
  props->fields.back().min_value  = 1.0f;
  props->fields.back().max_value  = 1000.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("scale", 100.0f);
  return true;
}
FilterPluginTable f_resize = {
  GUID(0x00016), FilterDefault, cutil::Str("リサイズ"), cutil::Str("リサイズ"), 0, "0", nullptr, nullptr, fn_init_resize, nullptr, fn_proc_resize, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
