#pragma once

#include <map>
#include <memory>
#include <movutl/plugin/stroke.hpp>
#include <string>
#include <vector>

namespace mu {

// 描画方式の管理。標準のものは初回アクセス時に登録され、プラグインは同じ経路(StrokePluginTable)で追加する
class StrokeRegistry {
public:
  static StrokeRegistry& Get();
  bool register_type(const StrokePluginTable& t); // 同名は上書き。名前が空/createが無いならfalse
  const StrokePluginTable* find(const std::string& name) const;
  std::unique_ptr<Stroke> create(const std::string& name, TiledImage* target, const BrushSettings& s) const; // 未登録ならnullptr
  std::vector<std::string> list() const;

private:
  StrokeRegistry();
  std::map<std::string, StrokePluginTable> types_;
};

// ブラシ先端の管理("circle"は標準)
class TipRegistry {
public:
  static TipRegistry& Get();
  bool register_type(const TipPluginTable& t);
  std::shared_ptr<BrushTip> create(const std::string& name, const cutil::Prop& params) const; // 未登録ならnullptr
  std::vector<std::string> list() const;

private:
  TipRegistry();
  std::map<std::string, TipPluginTable> types_;
};

} // namespace mu
