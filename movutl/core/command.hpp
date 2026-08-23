#pragma once

#include <cutil/ref.hpp>
#include <cutil/string.hpp>
#include <imgui.h>
#include <movutl/core/defines.hpp>
#include <vector>

namespace mu {

using cutil::Ref;

enum class CommandStatus {
  Running,  // 継続実行中。CommandManagerが毎フレームtick()を呼び続ける
  Finished, // 完了。実行中リストから外れる
  Failed,   // 失敗。実行中リストから外れる(エラーログはコマンド側で出す)
};

// 1つのコマンド(job)を表す基底クラス。継承して実装する(plugin向けの関数ポインタ登録APIは今回スコープ外)
class mCommand {
public:
  mCommand()          = default;
  virtual ~mCommand() = default;

  cutil::Str id;                          // run_command/has_command/cancel_commandで指定する一意なID
  cutil::Str name;                        // メニュー等に出す表示名
  cutil::Str description;                 // コマンドの内容説明
  ImGuiKeyChord shortcut = ImGuiKey_None; // 例: ImGuiKey_Space, ImGuiMod_Ctrl | ImGuiKey_S

  // run_command()から呼ばれる。Runningを返すとtick()が毎フレーム呼ばれ続ける
  virtual CommandStatus on_start() { return CommandStatus::Finished; }
  // Running状態の間、毎フレーム呼ばれる
  virtual CommandStatus tick() { return CommandStatus::Finished; }
  virtual void on_cancel() {}  // cancel_command()から呼ばれる
  virtual void on_undo() {}    // 将来のUndoスタック実装用フック(今回は中身なし)
  virtual void on_redo() {}    // 将来のRedoスタック実装用フック(今回は中身なし)
  virtual void update_ui() {}     // メニュー等にこのコマンド用のUIを描画したい場合に使うフック
  virtual void update_tool_ui() {} // ツールバー拡張用フック(今回は未使用)
};

class CommandManager {
public:
  MOVUTL_DECLARE_SINGLETON(CommandManager);
  CommandManager()  = default;
  ~CommandManager() = default;

  void register_command(const Ref<mCommand>& cmd);
  bool has_command(const char* id) const;
  bool run_command(const char* id);
  void cancel_command(const char* id);
  void tick_running_commands();

  std::vector<Ref<mCommand>> commands_;
  std::vector<Ref<mCommand>> running_;
};

bool run_command(const char* id);
bool has_command(const char* id);
void cancel_command(const char* id);

} // namespace mu
