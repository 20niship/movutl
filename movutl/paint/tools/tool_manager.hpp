#pragma once

#include <map>
#include <memory>
#include <movutl/paint/tools/tool.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mu {

// ツールの選択(キーでの順送り、押している間の一時切替)と、操作中の入力の振り分け
class ToolManager {
public:
  ToolManager(); // 標準ツールを登録する。最初はペン

  const std::vector<std::unique_ptr<Tool>>& tools() const { return tools_; }
  Tool* find(const std::string& id) const;                     // 無ければnullptr
  Tool& selected() const { return *selected_; }                // 選んでいるツール
  Tool& active() const { return temp_ ? *temp_ : *selected_; } // 実際に使われるツール(一時切替中はそれ)

  bool select(const std::string& id);
  bool select_by_key(const std::string& key); // 同じキーのツール中なら次へ、そうでなければそのキーで最後に使ったツールへ

  bool begin_temporary(const std::string& id); // 押している間の一時切替(Altのスポイト、ペンの消しゴム側)
  void end_temporary() { temp_ = nullptr; }
  bool temporary() const { return temp_ != nullptr; }

  // 操作。press時点のactive()が、途中でツールを切り替えてもrelease/cancelまで担当する
  bool press(const ToolContext& c, const PenSample& p); // 操作中は無視してfalse
  void drag(const PenSample& p);
  void tick(float dt);
  std::unique_ptr<PaintEdit> release(); // 取り消し用のEdit(無ければnullptr)
  void cancel();
  bool busy() const { return op_ != nullptr; }

private:
  std::vector<std::unique_ptr<Tool>> tools_;
  std::map<std::string, Tool*> last_; // キーごとに最後に使ったツール
  Tool* selected_ = nullptr;
  Tool* temp_     = nullptr;
  Tool* op_       = nullptr; // 操作中のツール
  std::optional<ToolContext> ctx_;
};

} // namespace mu
