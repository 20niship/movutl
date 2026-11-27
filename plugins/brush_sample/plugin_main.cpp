// ブラシプラグインのサンプル: Strokeを継承した自作の描画方式 "cross"(十字のスタンプを並べる)を登録する
#include <cstring>
#include <movutl/paint/stroke.hpp>
#include <movutl/plugin/abi.h>
#include <movutl/plugin/plugin.hpp>
#include <movutl/plugin/stroke.hpp>
#include <movutl/raster/blend.hpp>

namespace {

class CrossStroke : public mu::Stroke {
public:
  using Stroke::Stroke;
  void add(const mu::PenSample& p) override {
    const int arm = std::max(1, cutil::get_or<int32_t>(settings_.params, "arm", 3)); // 腕の長さ(px)
    stamp_along(p, [&](const mu::StampInfo& s) {
      const int cx = (int)s.x, cy = (int)s.y;
      for(int d = -arm; d <= arm; d++) {
        for(int axis = 0; axis < 2; axis++) {
          const int x = cx + (axis ? 0 : d), y = cy + (axis ? d : 0);
          if(!target_->in_range(x, y)) continue;
          const float cov = (clip_ ? clip_->at(x, y) : 1.0f) * s.flow;
          if(cov > 0.0f) mu::blend_over_straight(target_->at(x, y), settings_.color, cov * settings_.color[3] / 255.0f);
        }
      }
    });
  }
};

mu::Stroke* create_cross(mu::TiledImage* t, const mu::BrushSettings& s) { return new CrossStroke(t, s); }

bool init_cross(cutil::PropInfo* props, cutil::Prop* defaults) {
  props->fields.push_back(cutil::PropInfo::Field("arm", 0, cutil::prop_info_of<int32_t>()));
  props->fields.back().set_label("腕の長さ(px)");
  defaults->set<int32_t>("arm", 3);
  return true;
}

mu::StrokePluginTable g_cross;

void plugin_init(mu::ABIContext* abi) {
  std::strncpy(g_cross.name, "cross", sizeof(g_cross.name) - 1);
  std::strncpy(g_cross.label, "十字", sizeof(g_cross.label) - 1);
  std::strncpy(g_cross.information, "サンプルのブラシプラグイン(十字のスタンプ)", sizeof(g_cross.information) - 1);
  g_cross.create  = &create_cross;
  g_cross.fn_init = &init_cross;
  abi->register_stroke_plugin(&g_cross);
}
void plugin_exit(mu::ABIContext*) {}

} // namespace

extern "C" void plugin_entry(mu::ABIContext*, mu::PluginTable* table) {
  std::memset(table, 0, sizeof(*table));
  std::strncpy(table->name, "Brush Sample", sizeof(table->name) - 1);
  std::strncpy(table->description, "Strokeを継承した自作ブラシのサンプルプラグイン", sizeof(table->description) - 1);
  table->plugin_init = &plugin_init;
  table->plugin_exit = &plugin_exit;
}
