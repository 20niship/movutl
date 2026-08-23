#include <doctest/doctest.h>
#include <movutl/core/command.hpp>

using namespace mu;

namespace {

struct DummyCommand : mCommand {
  DummyCommand() {
    id   = "dummy_test_command";
    name = "dummy";
  }
  CommandStatus on_start() override {
    started = true;
    return CommandStatus::Finished;
  }
  void on_cancel() override { cancelled = true; }
  bool started   = false;
  bool cancelled = false;
};

struct RunningCommand : mCommand {
  RunningCommand() { id = "running_test_command"; }
  CommandStatus on_start() override { return CommandStatus::Running; }
  CommandStatus tick() override { return ++ticks >= 2 ? CommandStatus::Finished : CommandStatus::Running; }
  int ticks = 0;
};

} // namespace

TEST_CASE("CommandManager register/run/has/cancel") {
  auto cm = CommandManager::Get();

  auto dummy = cutil::make_ref<DummyCommand>();
  cm->register_command(dummy);
  CHECK(cm->has_command("dummy_test_command"));
  CHECK_FALSE(cm->has_command("no_such_command"));

  CHECK(cm->run_command("dummy_test_command"));
  CHECK(dummy->started);

  auto running = cutil::make_ref<RunningCommand>();
  cm->register_command(running);
  CHECK(cm->run_command("running_test_command"));
  CHECK(running->ticks == 0);
  cm->tick_running_commands();
  CHECK(running->ticks == 1);
  cm->tick_running_commands();
  CHECK(running->ticks == 2);

  cm->run_command("running_test_command");
  cm->cancel_command("running_test_command");
}
