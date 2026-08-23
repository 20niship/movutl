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
