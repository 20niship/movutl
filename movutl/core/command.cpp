#include <algorithm>
#include <movutl/core/command.hpp>
#include <movutl/core/logger.hpp>

namespace mu {

void CommandManager::register_command(const Ref<mCommand>& cmd) {
  if(has_command(cmd->id.c_str())) {
    LOG_F(WARNING, "CommandManager::register_command: id '%s' already registered, overwriting", cmd->id.c_str());
    cancel_command(cmd->id.c_str());
    commands_.erase(std::remove_if(commands_.begin(), commands_.end(), [&](const Ref<mCommand>& c) { return c->id == cmd->id; }), commands_.end());
  }
  commands_.push_back(cmd);
}

bool CommandManager::has_command(const char* id) const {
  for(const auto& c : commands_)
    if(c->id == id) return true;
  return false;
}

bool CommandManager::run_command(const char* id) {
  for(const auto& c : commands_) {
    if(c->id != id) continue;
    auto status = c->on_start();
    if(status == CommandStatus::Running) running_.push_back(c);
    return status != CommandStatus::Failed;
  }
  LOG_F(WARNING, "CommandManager::run_command: unknown id '%s'", id);
  return false;
}

void CommandManager::cancel_command(const char* id) {
  for(auto it = running_.begin(); it != running_.end(); ++it) {
    if((*it)->id != id) continue;
    (*it)->on_cancel();
    running_.erase(it);
    return;
  }
}

void CommandManager::tick_running_commands() {
  for(auto it = running_.begin(); it != running_.end();) {
    auto status = (*it)->tick();
    if(status == CommandStatus::Running) {
      ++it;
    } else {
      it = running_.erase(it);
    }
  }
}

bool run_command(const char* id) { return CommandManager::Get()->run_command(id); }
bool has_command(const char* id) { return CommandManager::Get()->has_command(id); }
void cancel_command(const char* id) { CommandManager::Get()->cancel_command(id); }

CommandManager* CommandManager::singleton_ = nullptr;

} // namespace mu
