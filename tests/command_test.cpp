#include <doctest/doctest.h>
#include <movutl/core/command.hpp>

using namespace mu;

namespace {

// run_command()の度に新しいインスタンスが作られるため、実行をまたぐ検証にはstaticメンバを使う
struct DummyCommand : mCommand {
  CommandStatus on_start() override {
    started = true;
    return CommandStatus::Finished;
  }
  void on_cancel() override { cancelled = true; }
  static bool started;
  static bool cancelled;
};
bool DummyCommand::started   = false;
bool DummyCommand::cancelled = false;

struct RunningCommand : mCommand {
  CommandStatus on_start() override { return CommandStatus::Running; }
  CommandStatus tick() override { return ++s_ticks >= 2 ? CommandStatus::Finished : CommandStatus::Running; }
  static int s_ticks;
};
int RunningCommand::s_ticks = 0;

// インスタンスメンバがrun_commandの度にリセットされる(=新しいインスタンスが使われる)ことを確認する
struct ReinitCommand : mCommand {
  CommandStatus on_start() override {
    CHECK_FALSE(started);
    started = true;
    return CommandStatus::Finished;
  }
  bool started = false;
};

} // namespace

TEST_CASE("CommandManager register/run/has/cancel") {
  register_command<DummyCommand>({"dummy_test_command", "dummy", "", ""});
  CHECK(has_command("dummy_test_command"));
  CHECK_FALSE(has_command("no_such_command"));

  CHECK(run_command("dummy_test_command"));
  CHECK(DummyCommand::started);

  register_command<RunningCommand>({"running_test_command", "", "", ""});
  CHECK(run_command("running_test_command"));
  CHECK(RunningCommand::s_ticks == 0);
  tick_running_commands();
  CHECK(RunningCommand::s_ticks == 1);
  tick_running_commands();
  CHECK(RunningCommand::s_ticks == 2);

  run_command("running_test_command");
  cancel_command("running_test_command");
}

TEST_CASE("CommandManager creates a fresh instance on every run") {
  register_command<ReinitCommand>({"reinit_test_command", "", "", ""});
  CHECK(run_command("reinit_test_command"));
  CHECK(run_command("reinit_test_command")); // 2回目もReinitCommand::startedがfalseから始まるはず
}

namespace {
// undoable()==trueなコマンド。on_undo/on_redoの呼び出し回数を記録する
struct UndoableCommand : mCommand {
  CommandStatus on_start() override {
    value += 1;
    return CommandStatus::Finished;
  }
  void on_undo() override { value -= 1; }
  void on_redo() override { value += 1; }
  bool undoable() const override { return true; }
  static int value;
};
int UndoableCommand::value = 0;

// undoable()==false(既定)のコマンド。undo履歴に積まれないことを確認する
struct NonUndoableCommand : mCommand {
  CommandStatus on_start() override {
    runs++;
    return CommandStatus::Finished;
  }
  static int runs;
};
int NonUndoableCommand::runs = 0;
} // namespace

TEST_CASE("undo_command/redo_command: undoable()なコマンドの履歴を取り消し/やり直しできる") {
  register_command<UndoableCommand>({"undoable_test_command", "", "", ""});
  UndoableCommand::value = 0;

  CHECK(run_command("undoable_test_command"));
  CHECK(UndoableCommand::value == 1);

  CHECK(undo_command());
  CHECK(UndoableCommand::value == 0);

  CHECK(redo_command());
  CHECK(UndoableCommand::value == 1);
}

TEST_CASE("undo_command: undoable()==falseなコマンドは履歴に積まれない") {
  register_command<NonUndoableCommand>({"non_undoable_test_command", "", "", ""});
  while(can_undo()) undo_command(); // CommandManagerはプロセス全体で共有のシングルトンのため、他テストの履歴が残っていても影響を受けないようクリアする
  NonUndoableCommand::runs = 0;

  CHECK(run_command("non_undoable_test_command"));
  CHECK(NonUndoableCommand::runs == 1);
  CHECK_FALSE(can_undo()); // 直前の履歴に積まれていないので取り消せない
}

TEST_CASE("run_command: 新規実行はredo履歴をクリアする") {
  register_command<UndoableCommand>({"undoable_test_command2", "", "", ""});
  UndoableCommand::value = 0;

  CHECK(run_command("undoable_test_command2"));
  CHECK(undo_command());
  CHECK(can_redo());

  CHECK(run_command("undoable_test_command2")); // 新規実行でredo履歴が消える
  CHECK_FALSE(can_redo());
}
