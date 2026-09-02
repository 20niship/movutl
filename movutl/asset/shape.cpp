#include <opencv2/opencv.hpp>
//
#include <cmath>
#include <cutil/prop.hpp>
#include <limits>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/core/prop_types.hpp>
#include <sstream>

namespace mu {

namespace {

std::vector<cv::Point2f> polygon_points(int32_t type, float w, float h) {
  switch(type) {
    case ShapeType_Triangle: return {{w / 2, 0}, {w, h}, {0, h}};
    case ShapeType_Hexagon: {
      std::vector<cv::Point2f> pts;
      for(int i = 0; i < 6; i++) {
        float a = (float)i / 6.0f * 2.0f * (float)M_PI - (float)M_PI / 2.0f;
        pts.push_back({w / 2 + w / 2 * std::cos(a), h / 2 + h / 2 * std::sin(a)});
      }
      return pts;
    }
    case ShapeType_Rect:
    default: return {{0, 0}, {w, 0}, {w, h}, {0, h}};
  }
}

// "x,y;x,y;..." 形式のテキストを点列にパースする
std::vector<cv::Point2f> parse_custom_path(const std::string& s) {
  std::vector<cv::Point2f> pts;
  std::stringstream ss(s);
  std::string seg;
  while(std::getline(ss, seg, ';')) {
    auto comma = seg.find(',');
    if(comma == std::string::npos) continue;
    try {
      float x = std::stof(seg.substr(0, comma));
      float y = std::stof(seg.substr(comma + 1));
      pts.push_back({x, y});
    } catch(...) {
      continue;
    }
  }
  return pts;
}

} // namespace

void ShapeEntt::re_render_image() {
  bool need = !img_ || last_type_ != shape_type_ || last_size_ != size_ || last_color_ != color_ || last_path_ != custom_path || last_border_color_ != border_color_ || last_border_width_ != border_width_;
  if(!need) return;
  last_type_         = shape_type_;
  last_size_         = size_;
  last_color_        = color_;
  last_path_         = custom_path;
  last_border_color_ = border_color_;
  last_border_width_ = border_width_;
  shape_offset_      = Vec2(0, 0);

  std::vector<cv::Point2f> pts;
  float canvas_w, canvas_h;

  if(shape_type_ == ShapeType_Custom) {
    pts = parse_custom_path(custom_path);
    if(pts.size() < 3) {
      img_.reset();
      return;
    }
    float minx = std::numeric_limits<float>::max(), maxx = std::numeric_limits<float>::lowest();
    float miny = std::numeric_limits<float>::max(), maxy = std::numeric_limits<float>::lowest();
    for(auto& p : pts) {
      minx = std::min(minx, p.x);
      maxx = std::max(maxx, p.x);
      miny = std::min(miny, p.y);
      maxy = std::max(maxy, p.y);
    }
    shape_offset_ = Vec2(minx, miny);
    canvas_w      = std::max(1.0f, maxx - minx);
    canvas_h      = std::max(1.0f, maxy - miny);
    for(auto& p : pts) {
      p.x -= minx;
      p.y -= miny;
    }
  } else {
    canvas_w = std::max(1.0f, size_[0]);
    canvas_h = std::max(1.0f, size_[1]);
    if(shape_type_ != ShapeType_Circle) pts = polygon_points(shape_type_, canvas_w, canvas_h);
  }

  // 枠線がキャンバス外にはみ出さないようborder_width分だけ余白を確保する
  int pad = std::max(0, border_width_);
  for(auto& p : pts) {
    p.x += pad;
    p.y += pad;
  }
  shape_offset_[0] -= pad;
  shape_offset_[1] -= pad;

  if(!img_) img_ = cutil::make_ref<Image>();
  img_->resize((int)std::ceil(canvas_w) + pad * 2, (int)std::ceil(canvas_h) + pad * 2);
  img_->has_alpha = true;
  img_->fill(0);

  cv::Mat mat((int)img_->height, (int)img_->width, CV_8UC4, img_->data());
  cv::Scalar col(color_[0], color_[1], color_[2], color_[3]);
  cv::Scalar border_col(border_color_[0], border_color_[1], border_color_[2], border_color_[3]);
  if(shape_type_ == ShapeType_Circle) {
    cv::Point center(mat.cols / 2, mat.rows / 2);
    cv::Size axes((int)std::ceil(canvas_w) / 2, (int)std::ceil(canvas_h) / 2);
    cv::ellipse(mat, center, axes, 0, 0, 360, col, cv::FILLED, cv::LINE_AA);
    if(border_width_ > 0) cv::ellipse(mat, center, axes, 0, 0, 360, border_col, border_width_, cv::LINE_AA);
  } else {
    std::vector<cv::Point> pts_i;
    pts_i.reserve(pts.size());
    for(auto& p : pts) pts_i.push_back(cv::Point((int)std::round(p.x), (int)std::round(p.y)));
    std::vector<std::vector<cv::Point>> polys{pts_i};
    cv::fillPoly(mat, polys, col, cv::LINE_AA);
    if(border_width_ > 0) cv::polylines(mat, polys, true, border_col, border_width_, cv::LINE_AA);
  }
}

bool ShapeEntt::render(Composition* cmp, Image* target, int frame) {
  (void)frame;
  re_render_image();
  if(!img_ || img_->empty() || !cmp || !target) return false;
  Vec2d pmin(pos_[0] + shape_offset_[0], pos_[1] + shape_offset_[1]);
  float rot_deg = rot_ * 180.0f / (float)M_PI;
  img_->copyto(target, pmin, 1.0f, rot_deg, alpha_ / 255.0f, trk.blend_);
  return true;
}

Ref<ShapeEntt> ShapeEntt::Create(const char* name, ShapeType type) {
  auto ent         = Ref<ShapeEntt>(new ShapeEntt());
  ent->name        = name;
  ent->shape_type_ = type;
  return ent;
}

} // namespace mu
