#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <movutl/core/props.hpp>

namespace mu {

class Entity;

// Undoコマンドのベースクラス
// Blenderのoperatorシステムを参考に、実行と取り消しの操作をカプセル化する
class UndoCommand {
public:
  virtual ~UndoCommand() = default;
  
  // コマンドを実行する（Redo用）
  virtual void execute() = 0;
  
  // コマンドを元に戻す（Undo用）
  virtual void undo() = 0;
  
  // コマンドの説明を取得
  virtual std::string description() const = 0;
};

// Entityのプロパティ変更を記録するコマンド
// getProps/setPropsを使用してUIの値変更を元に戻す
class EntityPropertyChangeCommand : public UndoCommand {
private:
  Entity* entity_;              // 対象のエンティティ
  Props old_props_;             // 変更前のプロパティ値
  Props new_props_;             // 変更後のプロパティ値
  std::string property_name_;   // 変更されたプロパティ名
  std::string entity_name_;     // エンティティ名（デバッグ用）
  
public:
  // コンストラクタ
  // entity: 対象のエンティティ
  // property_name: 変更されるプロパティの名前
  // old_value: 変更前の値
  // new_value: 変更後の値
  EntityPropertyChangeCommand(
    Entity* entity,
    const std::string& property_name,
    const Props::Value& old_value,
    const Props::Value& new_value
  );
  
  virtual ~EntityPropertyChangeCommand() = default;
  
  void execute() override;
  void undo() override;
  std::string description() const override;
};

// Undo/Redoスタックを管理するマネージャークラス
// Blenderのように複数の操作を記録し、undo/redoを実現する
class UndoManager {
private:
  std::vector<std::unique_ptr<UndoCommand>> undo_stack_;  // Undoスタック
  std::vector<std::unique_ptr<UndoCommand>> redo_stack_;  // Redoスタック
  size_t max_stack_size_ = 100;  // スタックの最大サイズ
  
public:
  UndoManager() = default;
  ~UndoManager() = default;
  
  // コマンドを実行してUndoスタックに追加
  void execute_command(std::unique_ptr<UndoCommand> command);
  
  // コマンドを実行せずにUndoスタックに追加（すでに実行済みの場合）
  void add_command(std::unique_ptr<UndoCommand> command);
  
  // 最後のコマンドを取り消す
  bool undo();
  
  // 最後に取り消したコマンドを再実行
  bool redo();
  
  // Undoスタックをクリア
  void clear();
  
  // Undoが可能かどうか
  bool can_undo() const { return !undo_stack_.empty(); }
  
  // Redoが可能かどうか
  bool can_redo() const { return !redo_stack_.empty(); }
  
  // Undoスタックのサイズを取得
  size_t undo_stack_size() const { return undo_stack_.size(); }
  
  // Redoスタックのサイズを取得
  size_t redo_stack_size() const { return redo_stack_.size(); }
  
  // 最大スタックサイズを設定
  void set_max_stack_size(size_t size) { max_stack_size_ = size; }
  
  // 最後のUndoコマンドの説明を取得
  std::string get_last_undo_description() const;
  
  // 最後のRedoコマンドの説明を取得
  std::string get_last_redo_description() const;
};

// グローバルなUndoマネージャーのインスタンスを取得
UndoManager& GetUndoManager();

} // namespace mu
