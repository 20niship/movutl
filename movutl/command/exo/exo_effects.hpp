#pragma once
#include <map>
#include <movutl/asset/entity.hpp>
#include <string>

namespace mu {

// exoのiniセクション(key=value)。exo_import.cppのSectionと同一型
using ExoSection = std::map<std::string, std::string>;

// exoの合成モード番号をBlendTypeへ変換する。対応が無い番号(輝度/色差/陰影/明暗/差分)はBlend_Alphaを返し*supportedをfalseにする
BlendType exo_blend_type(int v, bool* supported);

// 標準描画セクションのblend=をEntity::blend_へ反映する(未対応の合成モードは取り込みレポートに記録)
void apply_exo_blend(Entity& e, const ExoSection* draw);

} // namespace mu
