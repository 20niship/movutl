#include <algorithm>
#include <movutl/core/command.hpp>
#include <movutl/core/logger.hpp>

namespace mu {

namespace {

struct CommandEntry {
  CommandInfo info;
  std::function<Ref<mCommand>()> factory;
};

struct RunningCommand {
  std::string id;
  Ref<mCommand> instance;
};

// CommandManagerは外部から直接使わないのでcommand.cppに閉じ、外部にはregister_command等の自由関数のみ公開する
class CommandManager {
public:
  MOVUTL_DECLARE_SINGLETON(CommandManager);
  CommandManager()  = default;
  ~CommandManager() = default;

  void register_command(CommandInfo info, std::function<Ref<mCommand>()> factory);
  bool has_command(const char* id) const;
  bool run_command(const char* id);
  void cancel_command(const char* id);
  void tick_running_commands();
  const std::vector<CommandInfo>& infos() const { return infos_; }

  bool undo();
  bool redo();
  bool can_undo() const { return !undo_stack_.empty(); }
  bool can_redo() const { return !redo_stack_.empty(); }

private:
  std::vector<CommandEntry> entries_;
  std::vector<RunningCommand> running_;
  std::vector<CommandInfo> infos_;
  // 実行済みコマンドの履歴。undoable()なコマンドのみ積む。新規実行時にredo_stack_はクリアする
  std::vector<Ref<mCommand>> undo_stack_;
  std::vector<Ref<mCommand>> redo_stack_;
};

void CommandManager::register_command(CommandInfo info, std::function<Ref<mCommand>()> factory) {
  if(has_command(info.id.c_str())) {
    LOG_F(WARNING, "CommandManager::register_command: id '%s' already registered, overwriting", info.id.c_str());
    cancel_command(info.id.c_str());
    entries_.erase(std::remove_if(entries_.begin(), entries_.end(), [&](const CommandEntry& e) { return e.info.id == info.id; }), entries_.end());
    infos_.erase(std::remove_if(infos_.begin(), infos_.end(), [&](const CommandInfo& i) { return i.id == info.id; }), infos_.end());
  }
  infos_.push_back(info);
  entries_.push_back(CommandEntry{std::move(info), std::move(factory)});
}

bool CommandManager::has_command(const char* id) const {
  for(const auto& e : entries_)
    if(e.info.id == id) return true;
  return false;
}

bool CommandManager::run_command(const char* id) {
  for(const auto& e : entries_) {
    if(e.info.id != id) continue;
    auto instance = e.factory();
    auto status   = instance->on_start();
    if(status == CommandStatus::Running) running_.push_back(RunningCommand{e.info.id, instance});
    if(status != CommandStatus::Failed && instance->undoable()) {
      undo_stack_.push_back(instance);
      redo_stack_.clear();
    }
    return status != CommandStatus::Failed;
  }
  LOG_F(WARNING, "CommandManager::run_command: unknown id '%s'", id);
  return false;
}

bool CommandManager::undo() {
  if(undo_stack_.empty()) return false;
  auto cmd = undo_stack_.back();
  undo_stack_.pop_back();
  cmd->on_undo();
  redo_stack_.push_back(cmd);
  return true;
}

bool CommandManager::redo() {
  if(redo_stack_.empty()) return false;
  auto cmd = redo_stack_.back();
  redo_stack_.pop_back();
  cmd->on_redo();
  undo_stack_.push_back(cmd);
  return true;
}

void CommandManager::cancel_command(const char* id) {
  for(auto it = running_.begin(); it != running_.end(); ++it) {
    if(it->id != id) continue;
    it->instance->on_cancel();
    running_.erase(it);
    return;
  }
}

void CommandManager::tick_running_commands() {
  for(auto it = running_.begin(); it != running_.end();) {
    auto status = it->instance->tick();
    if(status == CommandStatus::Running) {
      ++it;
    } else {
      it = running_.erase(it);
    }
  }
}

CommandManager* CommandManager::singleton_ = nullptr;

} // namespace

void register_command(CommandInfo info, std::function<Ref<mCommand>()> factory) { CommandManager::Get()->register_command(std::move(info), std::move(factory)); }
bool run_command(const char* id) { return CommandManager::Get()->run_command(id); }
bool has_command(const char* id) { return CommandManager::Get()->has_command(id); }
void cancel_command(const char* id) { CommandManager::Get()->cancel_command(id); }
void tick_running_commands() { CommandManager::Get()->tick_running_commands(); }
const std::vector<CommandInfo>& get_command_infos() { return CommandManager::Get()->infos(); }
bool undo_command() { return CommandManager::Get()->undo(); }
bool redo_command() { return CommandManager::Get()->redo(); }
bool can_undo() { return CommandManager::Get()->can_undo(); }
bool can_redo() { return CommandManager::Get()->can_redo(); }

} // namespace mu
