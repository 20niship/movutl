#include <cstdlib>
#include <movutl/command/exo/exo_effects.hpp>
#include <movutl/command/exo/exo_report.hpp>

namespace mu {

namespace {
const char* const kExoBlendNames[] = {"通常", "加算", "減算", "乗算", "スクリーン", "オーバーレイ", "比較(明)", "比較(暗)", "輝度", "色差", "陰影", "明暗", "差分"};
}

BlendType exo_blend_type(int v, bool* supported) {
  if(supported) *supported = true;
  switch(v) {
    case 0: return Blend_Alpha;
    case 1: return Blend_Add;
    case 2: return Blend_Sub;
    case 3: return Blend_Mul;
    case 4: return Blend_Screen;
    case 5: return Blend_Overlay;
    case 6: return Blend_Lighten;
    case 7: return Blend_Darken;
    default:
      if(supported) *supported = false;
      return Blend_Alpha;
  }
}

void apply_exo_blend(Entity& e, const ExoSection* draw) {
  if(!draw) return;
  auto it = draw->find("blend");
  if(it == draw->end() || it->second.empty()) return;
  int v          = atoi(it->second.c_str());
  bool supported = true;
  e.blend_       = exo_blend_type(v, &supported);
  if(!supported) {
    std::string name = (v >= 0 && v < (int)(sizeof(kExoBlendNames) / sizeof(*kExoBlendNames))) ? kExoBlendNames[v] : std::to_string(v);
    exo_import_report().add("合成モード「" + name + "」は未対応のため通常で代用しました");
  }
}

} // namespace mu
