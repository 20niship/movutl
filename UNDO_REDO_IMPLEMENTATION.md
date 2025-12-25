# Undo/Redoシステム実装ドキュメント

## 概要

このPRでは、pygenをベースとした編集UI（propgen）に対して、Blenderのoperatorシステムを参考にしたundo/redoスタックを実装しました。

## 実装方針

### 基本設計

1. **コマンドパターンの採用**: 各操作をコマンドオブジェクトとしてカプセル化
2. **スタックベースの管理**: Undo/Redoスタックで履歴を管理
3. **Props APIの活用**: Entity::getProps/setPropsを使用して値の変更を記録・復元

### Blenderのoperatorシステムとの類似点

- コマンドの実行と取り消しを分離
- スタックベースの履歴管理
- コマンドごとの説明文（description）
- 最大スタックサイズの制限

## 実装内容

### 1. コアシステム（movutl/core/undo.hpp, undo.cpp）

#### UndoCommand（基底クラス）

```cpp
class UndoCommand {
public:
  virtual void execute() = 0;      // コマンドを実行（Redo用）
  virtual void undo() = 0;         // コマンドを元に戻す（Undo用）
  virtual std::string description() const = 0;  // コマンドの説明
};
```

#### EntityPropertyChangeCommand

Entityのプロパティ変更を記録するコマンドクラス。

```cpp
EntityPropertyChangeCommand(
  Entity* entity,                    // 対象のエンティティ
  const std::string& property_name,  // 変更されるプロパティの名前
  const Props::Value& old_value,     // 変更前の値
  const Props::Value& new_value      // 変更後の値
);
```

**動作**:
- `execute()`: new_valueをentity->setProps()で設定
- `undo()`: old_valueをentity->setProps()で復元

#### UndoManager

Undo/Redoスタックを管理するシングルトンクラス。

```cpp
UndoManager& mgr = GetUndoManager();

// コマンドを実行してスタックに追加
mgr.execute_command(std::move(command));

// コマンドを実行せずにスタックに追加（すでに実行済みの場合）
mgr.add_command(std::move(command));

// 元に戻す
mgr.undo();

// やり直す
mgr.redo();

// スタックをクリア
mgr.clear();

// 状態確認
bool can_undo = mgr.can_undo();
bool can_redo = mgr.can_redo();
std::string desc = mgr.get_last_undo_description();
```

### 2. UI統合（movutl/gui/widgets.cpp）

#### wd_entt_props_editor関数の変更

UIウィジェットでの値変更を検知し、undoコマンドを自動生成します。

**検知ロジック**:
1. ウィジェットにフォーカスが当たった時、元の値を記録
2. ウィジェットからフォーカスが外れた時、値の変更をチェック
3. 値が変更されていた場合、EntityPropertyChangeCommandを作成してスタックに追加

```cpp
// フォーカスが当たった時（編集開始）
if(focus_id != last_focus_id && focus_id == item_id_ && focus_id != 0) {
  last_property_value = p.get_(pi.name);
  last_property_name = pi.name;
  last_entity = e;
}

// フォーカスが外れた時（編集終了）
if(item_id_ == last_focus_id && last_focus_id != 0) {
  if(focus_id != item_id_) {
    v_comfirmed = p.get_(pi.name);
    
    // 値が変更されているかチェック
    bool value_changed = (last_property_value != v_comfirmed);
    
    // 変更されていたらUndoコマンドを作成
    if(value_changed) {
      auto undo_cmd = std::make_unique<EntityPropertyChangeCommand>(
        e, pi.name, last_property_value, v_comfirmed
      );
      GetUndoManager().add_command(std::move(undo_cmd));
    }
  }
}
```

### 3. メニュー統合（movutl/gui/mainmenu.cpp）

「編集」メニューにUndo/Redoメニュー項目を追加:

- **元に戻す** (Ctrl+Z): 最後の操作を取り消す
- **やり直す** (Ctrl+Y): 最後に取り消した操作を再実行
- **履歴をクリア**: Undo/Redoスタックをクリア

メニュー項目にマウスオーバーすると、コマンドの説明がツールチップで表示されます。

### 4. キーボードショートカット（movutl/gui/window.cpp）

グローバルなキーボードショートカットを実装:

- **Ctrl+Z**: Undo
- **Ctrl+Y**: Redo
- **Ctrl+Shift+Z**: Redo（代替）

```cpp
// Ctrl+Z: Undo
if(io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
  auto& undo_mgr = GetUndoManager();
  if(undo_mgr.can_undo()) {
    undo_mgr.undo();
  }
}

// Ctrl+Y or Ctrl+Shift+Z: Redo
if(io.KeyCtrl && (ImGui::IsKeyPressed(ImGuiKey_Y, false) || 
                  (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false)))) {
  auto& undo_mgr = GetUndoManager();
  if(undo_mgr.can_redo()) {
    undo_mgr.redo();
  }
}
```

## APIの使い方

### 基本的な使い方

```cpp
#include <movutl/core/undo.hpp>

// グローバルなUndoマネージャーを取得
auto& undo_mgr = mu::GetUndoManager();

// Undo/Redoの実行
if(undo_mgr.can_undo()) {
  undo_mgr.undo();
}

if(undo_mgr.can_redo()) {
  undo_mgr.redo();
}

// 履歴のクリア
undo_mgr.clear();
```

### カスタムコマンドの作成

新しいコマンドタイプを作成する場合:

```cpp
class MyCustomCommand : public mu::UndoCommand {
private:
  // コマンドの状態を保存
  
public:
  void execute() override {
    // コマンドを実行
  }
  
  void undo() override {
    // コマンドを元に戻す
  }
  
  std::string description() const override {
    return "カスタムコマンドの説明";
  }
};

// 使用例
auto cmd = std::make_unique<MyCustomCommand>();
GetUndoManager().execute_command(std::move(cmd));
```

### プロパティ変更の記録（手動）

```cpp
Entity* entity = /* ... */;

// 変更前の値を取得
auto old_props = entity->getProps();
auto old_value = old_props.get_("property_name");

// プロパティを変更
Props new_props;
new_props["property_name"] = new_value;
entity->setProps(new_props);

// 変更後の値を取得
auto current_props = entity->getProps();
auto new_value = current_props.get_("property_name");

// Undoコマンドを作成（すでに実行済みなのでadd_commandを使用）
auto cmd = std::make_unique<EntityPropertyChangeCommand>(
  entity, "property_name", old_value, new_value
);
GetUndoManager().add_command(std::move(cmd));
```

## 注意事項

### メモリ管理

- EntityPropertyChangeCommandはEntity*ポインタを保持します
- Entityが削除された場合、コマンドの実行は安全にスキップされます
- エンティティのライフタイムには注意が必要です

### スタックサイズ

- デフォルトの最大スタックサイズは100です
- `set_max_stack_size()`で変更可能です

```cpp
GetUndoManager().set_max_stack_size(200);
```

### パフォーマンス

- 各コマンドはPropsオブジェクトのコピーを保持します
- 大量のプロパティを持つエンティティの場合、メモリ使用量に注意が必要です

## 今後の拡張可能性

1. **コマンドのグループ化**: 複数のコマンドを1つのUndoアクションとしてまとめる
2. **選択的Undo**: 特定のエンティティやプロパティのみをUndoする
3. **永続化**: Undo履歴をファイルに保存・復元
4. **マクロ記録**: 一連の操作を記録して再生可能にする

## テスト

UIからの実際の操作で動作確認:

1. エンティティのプロパティを変更
2. 「編集」→「元に戻す」またはCtrl+Zでプロパティが元の値に戻ることを確認
3. 「編集」→「やり直す」またはCtrl+Yで変更が再適用されることを確認
4. 複数の変更を行い、順次Undoできることを確認

## まとめ

このUndo/Redoシステムは、pygenベースのUI編集システムに対して、直感的で強力な操作履歴管理を提供します。Blenderのoperatorシステムを参考にした設計により、拡張性と保守性を兼ね備えた実装となっています。
