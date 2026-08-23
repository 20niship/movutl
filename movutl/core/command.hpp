#pragma once

#include <cutil/ref.hpp>
#include <functional>
#include <movutl/core/defines.hpp>
#include <string>
#include <vector>

namespace mu {

using cutil::Ref;

enum class CommandStatus {
  Running,  // 継続実行中。CommandManagerが毎フレームtick()を呼び続ける
  Finished, // 完了。実行中リストから外れる
  Failed,   // 失敗。実行中リストから外れる(エラーログはコマンド側で出す)
};

// 1つのコマンド(job)を表す基底クラス。run_command()の度に新しいインスタンスが生成されるため、実行をまたぐ状態はstaticメンバ等で持つこと
class mCommand {
public:
  mCommand()          = default;
  virtual ~mCommand() = default;

  // run_command()から呼ばれる。Runningを返すとtick()が毎フレーム呼ばれ続ける
  virtual CommandStatus on_start() { return CommandStatus::Finished; }
  // Running状態の間、毎フレーム呼ばれる
  virtual CommandStatus tick() { return CommandStatus::Finished; }
  virtual void on_cancel() {}      // cancel_command()から呼ばれる
  virtual void on_undo() {}        // 将来のUndoスタック実装用フック(今回は中身なし)
  virtual void on_redo() {}        // 将来のRedoスタック実装用フック(今回は中身なし)
  virtual void update_ui() {}      // メニュー等にこのコマンド用のUIを描画したい場合に使うフック
  virtual void update_tool_ui() {} // ツールバー拡張用フック(今回は未使用)
};

// コマンドの登録情報。mCommandのインスタンス(実行の度に生成/破棄される)とは分離して管理する
struct CommandInfo {
  std::string id;          // run_command/has_command/cancel_commandで指定する一意なID
  std::string name;        // メニュー等に出す表示名
  std::string description; // コマンドの内容説明
  std::string shortcut;    // vim風のキー表記(例: "ctrl+shift+a", "g g")。空文字ならショートカット無し。空白区切りで複数キーの連続入力になる
};

// コマンドを登録する。run_command()のたびにfactoryを呼んでmCommandの新しいインスタンスを生成する
void register_command(CommandInfo info, std::function<Ref<mCommand>()> factory);

// register_command<T>(info, args...): Tのコンストラクタ引数argsを保持し、実行の度にmake_ref<T>(args...)する糖衣構文
template <typename T, typename... Args> void register_command(CommandInfo info, Args... args) {
  register_command(std::move(info), [args...]() -> Ref<mCommand> { return cutil::make_ref<T>(args...); });
}

bool run_command(const char* id);
bool has_command(const char* id);
void cancel_command(const char* id);
void tick_running_commands();
const std::vector<CommandInfo>& get_command_infos(); // 登録済みコマンド一覧(メニュー表示やショートカット判定に使う)

} // namespace mu
