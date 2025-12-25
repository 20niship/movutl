#include <movutl/core/undo.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/logger.hpp>

namespace mu {

// EntityPropertyChangeCommandの実装

EntityPropertyChangeCommand::EntityPropertyChangeCommand(
  Entity* entity,
  const std::string& property_name,
  const Props::Value& old_value,
  const Props::Value& new_value
) : entity_(entity), property_name_(property_name) {
  if(entity_) {
    entity_name_ = entity_->name.c_str();
  }
  
  // 変更前後の値を保存
  old_props_.values[property_name] = old_value;
  new_props_.values[property_name] = new_value;
}

void EntityPropertyChangeCommand::execute() {
  if(!entity_) {
    LOG_F(WARNING, "EntityPropertyChangeCommand::execute() - entity is null");
    return;
  }
  
  // 新しい値を設定
  // 注: setPropsは指定されたプロパティのみを更新し、他のプロパティは保持されます
  entity_->setProps(new_props_);
  LOG_F(1, "Executed: %s", description().c_str());
}

void EntityPropertyChangeCommand::undo() {
  if(!entity_) {
    LOG_F(WARNING, "EntityPropertyChangeCommand::undo() - entity is null");
    return;
  }
  
  // 古い値を復元
  // 注: setPropsは指定されたプロパティのみを更新し、他のプロパティは保持されます
  entity_->setProps(old_props_);
  LOG_F(1, "Undone: %s", description().c_str());
}

std::string EntityPropertyChangeCommand::description() const {
  return "プロパティ変更: " + entity_name_ + "." + property_name_;
}

// UndoManagerの実装

void UndoManager::execute_command(std::unique_ptr<UndoCommand> command) {
  if(!command) {
    LOG_F(WARNING, "UndoManager::execute_command() - command is null");
    return;
  }
  
  // コマンドを実行
  command->execute();
  
  // Undoスタックに追加
  undo_stack_.push_back(std::move(command));
  
  // Redoスタックをクリア（新しい操作が実行されたため）
  redo_stack_.clear();
  
  // スタックサイズが上限を超えた場合、古いものから削除
  if(undo_stack_.size() > max_stack_size_) {
    undo_stack_.erase(undo_stack_.begin());
  }
  
  LOG_F(1, "Command executed. Undo stack size: %zu", undo_stack_.size());
}

void UndoManager::add_command(std::unique_ptr<UndoCommand> command) {
  if(!command) {
    LOG_F(WARNING, "UndoManager::add_command() - command is null");
    return;
  }
  
  // コマンドを実行せずにスタックに追加（すでに実行済みと仮定）
  undo_stack_.push_back(std::move(command));
  
  // Redoスタックをクリア（新しい操作が実行されたため）
  redo_stack_.clear();
  
  // スタックサイズが上限を超えた場合、古いものから削除
  if(undo_stack_.size() > max_stack_size_) {
    undo_stack_.erase(undo_stack_.begin());
  }
  
  LOG_F(1, "Command added to stack. Undo stack size: %zu", undo_stack_.size());
}

bool UndoManager::undo() {
  if(undo_stack_.empty()) {
    LOG_F(1, "Undo stack is empty");
    return false;
  }
  
  // 最後のコマンドを取得
  auto command = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  
  // Undoを実行
  command->undo();
  
  // Redoスタックに移動
  redo_stack_.push_back(std::move(command));
  
  LOG_F(1, "Undo executed. Undo stack size: %zu, Redo stack size: %zu", 
        undo_stack_.size(), redo_stack_.size());
  
  return true;
}

bool UndoManager::redo() {
  if(redo_stack_.empty()) {
    LOG_F(1, "Redo stack is empty");
    return false;
  }
  
  // 最後のコマンドを取得
  auto command = std::move(redo_stack_.back());
  redo_stack_.pop_back();
  
  // Redoを実行
  command->execute();
  
  // Undoスタックに移動
  undo_stack_.push_back(std::move(command));
  
  LOG_F(1, "Redo executed. Undo stack size: %zu, Redo stack size: %zu", 
        undo_stack_.size(), redo_stack_.size());
  
  return true;
}

void UndoManager::clear() {
  undo_stack_.clear();
  redo_stack_.clear();
  LOG_F(1, "Undo/Redo stacks cleared");
}

std::string UndoManager::get_last_undo_description() const {
  if(undo_stack_.empty()) {
    return "";
  }
  return undo_stack_.back()->description();
}

std::string UndoManager::get_last_redo_description() const {
  if(redo_stack_.empty()) {
    return "";
  }
  return redo_stack_.back()->description();
}

// グローバルなUndoマネージャーのインスタンス
static UndoManager g_undo_manager;

UndoManager& GetUndoManager() {
  return g_undo_manager;
}

} // namespace mu
