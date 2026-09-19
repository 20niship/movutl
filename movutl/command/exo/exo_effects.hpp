#pragma once
#include <map>
#include <movutl/asset/entity.hpp>
#include <movutl/core/anim.hpp>
#include <string>
#include <vector>

namespace mu {

class Composition;

// exoのiniセクション(key=value)。exo_import.cppのSectionと同一型
using ExoSection = std::map<std::string, std::string>;

// exoの合成モード番号をBlendTypeへ変換する。対応が無い番号(輝度/色差/陰影/明暗/差分)はBlend_Alphaを返し*supportedをfalseにする
BlendType exo_blend_type(int v, bool* supported);

// 標準描画セクションのblend=をEntity::blend_へ反映する(未対応の合成モードは取り込みレポートに記録)
void apply_exo_blend(Entity& e, const ExoSection* draw);

// オブジェクトセクションの camera=(カメラ制御の対象) / clipping=(上のオブジェクトでクリッピング) / overlay=0 を反映する
void apply_exo_object_flags(Entity& e, const ExoSection& obj);

// [exedit]のwidth/height/rate/scale/audio_rate/audio_chをCompositionへ反映する。既存Entityがある(=既存プロジェクトへ追加取り込みする)場合は変更しない
void apply_exo_header(Composition& comp, const ExoSection& exedit);

// トラックバー値 "開始値,終了値,移動方式[,パラメータ]" (移動しない値は単一の数値)
struct ExoTrack {
  float start = 0.f;
  float end   = 0.f;
  int mode    = 0;
  bool animated() const { return mode != 0 && start != end; }
};
ExoTrack parse_exo_track(const std::string& v);

// exoの移動方式(1:直線 2:曲線 3:瞬間 4:中間点無視 5:移動量指定 6:ランダム 7:加減速)に最も近い補間を返す。*exactがfalseなら近似
AniInterpType exo_track_interp(int mode, bool* exact);

// 標準描画/拡張描画/標準再生以外のエフェクト([N.1]以降)を対応表に従ってフィルタとして追加する。対応表に無いエフェクトや未対応パラメータは取り込みレポートに記録する。
// キーフレームはEntityのfstart_/fend_(絶対フレーム)へ打つためset_range後に呼ぶこと
void apply_exo_effects(Entity& e, const std::vector<const ExoSection*>& effects);

// 標準描画/標準再生のトラックバーが開始値と終了値で変化する場合、Entityの変換・音量はアニメーション未対応のため先頭値のまま取り込んだことをレポートに記録する
void report_exo_unanimated_tracks(const ExoSection* draw, const ExoSection* play);

} // namespace mu
