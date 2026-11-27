#pragma once

#include <map>
#include <movutl/paint/brush.hpp>
#include <string>
#include <vector>

namespace mu {

// サブツール1つ。操作で変わるのはcurrentで、reset()で初期値(initial)に戻せる
struct BrushPreset {
  std::string name;
  std::string group;        // サブツールパレットのタブ名
  std::string tool;         // ToolのID
  std::string pattern_file; // "pattern"で貼る画像(assets/brushes/内のファイル名)。BrushSettings::patternはここから読む
  BrushSettings initial, current;
  void reset() { current = initial; }
};

// サブツール一覧と、ツールごとの選択状態
class BrushLibrary {
public:
  std::vector<BrushPreset> presets;

  static BrushLibrary standard(); // 標準サブツール

  std::vector<BrushPreset*> of_tool(const std::string& tool);
  BrushPreset* selected(const std::string& tool); // 未選択なら最初のもの。そのツールのプリセットが無ければnullptr
  bool select(const std::string& tool, const std::string& name);

  // サブツールの編集。呼ぶとBrushPreset*は無効になる
  bool duplicate(const std::string& tool, const std::string& name);     // 直後に複製して選ぶ
  bool remove(const std::string& tool, const std::string& name);        // ツール最後の1つは消せない
  bool move(const std::string& tool, const std::string& name, int dir); // 同じグループ内で1つ前(dir<0)/後(dir>0)へ

  // JSON保存/読込。読込に失敗した場合はfalseで内容は変わらない。描画色と画像(tip, texture.img)は保存しない
  std::string dump() const; // save()が書くJSON。変更の検出にも使う
  bool save(const std::string& path) const;
  bool load(const std::string& path);

private:
  std::map<std::string, std::string> selected_; // tool → preset name
};

} // namespace mu
