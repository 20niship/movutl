#pragma once
#include <cutil/prop.hpp>
#include <memory>
#include <movutl/paint/brush.hpp>
#include <movutl/paint/stroke.hpp>

namespace mu {

// 描画方式(Strokeの派生クラス)を登録するテーブル。プラグインはplugin_initでABIContext::register_stroke_pluginに渡す
struct StrokePluginTable {
  char name[64]                                                 = {};      // BrushSettings::engineで指定する名前
  char label[64]                                                = {};      // UI表示名
  char information[256]                                         = {};      // 説明
  Stroke* (*create)(TiledImage* target, const BrushSettings& s) = nullptr; // newしたStrokeを返す(所有権は呼び出し側)
  // ブラシ固有パラメータをpropsに宣言し、defaultsに初期値を入れる(NULLなら無し)。値はBrushSettings::paramsに入る
  bool (*fn_init)(cutil::PropInfo* props, cutil::Prop* defaults) = nullptr;
  cutil::PropInfo props;
  cutil::Prop defaults;
};

// ブラシ先端(BrushTipの派生クラス)を登録するテーブル
struct TipPluginTable {
  char name[64]                                                  = {};
  char label[64]                                                 = {};
  std::shared_ptr<BrushTip> (*create)(const cutil::Prop& params) = nullptr;
};

} // namespace mu
