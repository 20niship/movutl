#include <algorithm>
#include <cmath>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_tile_filter.hpp>
#include <vector>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// sx=(x*nx)%w, sy=(y*ny)%h (出力=元サイズの前提でcellw割りが約分できる整数式)。浮動小数点のx/cellwはCPU/GPUで丸め方向が食い違うため使わない
bool fn_proc_tile(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Tile::fn_proc");

  const int nx = std::max(1, cutil::get_or<int32_t>(p, "nx", 2));
  const int ny = std::max(1, cutil::get_or<int32_t>(p, "ny", 2));
  Image* img   = fpip->img;
  const int w = (int)img->width, h = (int)img->height;
  if(w <= 0 || h <= 0 || (nx == 1 && ny == 1)) return true;

  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);
  Vec4b* dst = img->data();
#pragma omp parallel for schedule(static)
  for(long y = 0; y < h; y++) {
    const int sy = (int)((y * (long)ny) % h);
    for(int x = 0; x < w; x++) {
      const int sx           = (x * nx) % w;
      dst[(size_t)y * w + x] = src[(size_t)sy * w + sx];
    }
  }
  return true;
}
bool fn_init_tile(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("nx", 0, cutil::prop_info_of<int32_t>()));
  props->fields.back().set_label("横の個数");
  props->fields.back().min_value = 1.0f;
  props->fields.back().max_value = 32.0f;
  defaults->set<int32_t>("nx", 2);

  props->fields.push_back(cutil::PropInfo::Field("ny", 0, cutil::prop_info_of<int32_t>()));
  props->fields.back().set_label("縦の個数");
  props->fields.back().min_value = 1.0f;
  props->fields.back().max_value = 32.0f;
  defaults->set<int32_t>("ny", 2);
  return true;
}
FilterPluginTable f_tile = {
  GUID(0x00030), FilterDefault, cutil::Str("並べて配置"), cutil::Str("並べて配置"), 0, "0", nullptr, nullptr, fn_init_tile, nullptr, fn_proc_tile, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
