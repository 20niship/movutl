# pygenで足りない機能の分析と実装方針

このドキュメントでは、UPROPERTYのようなリフレクション、自動UI生成、シリアライズ/デシリアライズ機能を実現するために、現状のpygen実装に足りない機能をまとめます。

## 現状の実装状況

### ⚠️ 既存コードの改善点
以下の軽微な問題が既存コードに存在します：
- `pygen_types.py`の`MArgument.detault`はタイポ（`default`が正しい）
- `pygen_types.py`の`namespace :str`は不要なスペースがある（`namespace: str`が正しい）

これらは後方互換性のため、新機能実装時に合わせて修正することを推奨します。

### ✅ 実装済みの機能

1. **基本的なプロパティ情報の抽出** (`main.py`, `pygen_types.py`)
   - MPROPERTYマクロからメタデータを読み取り
   - 型情報、デフォルト値、min/max/step値の取得
   - カテゴリ、説明文、表示名の取得

2. **Luaバインディング自動生成** (`LuaintfWriter.py`, `LuaTypeWriter.py`)
   - クラス、関数、enumのLua公開
   - LuaIntfを使用したバインディングコード生成

3. **PropsInfoによる基本的なUI生成サポート** (`PropsWriter.py`)
   - `getPropsInfo()`: プロパティのメタデータを返す
   - `getProps()`: 現在の値を取得
   - `setProps()`: 値を設定

4. **JSONシリアライズ/デシリアライズ** (`props.hpp`, `json_loadsave.cpp`)
   - Props型のJSON変換機能
   - ファイルの読み書き対応

### ❌ 不足している機能

以下、UPROPERTYシステムと比較して不足している機能を列挙します。

---

## 1. クラス階層のリフレクション機能

### 現状の問題
- クラスの継承関係が記録されていない
- 基底クラスのプロパティを継承クラスで取得できない
- クラス名からインスタンスを動的に生成できない

### 必要な機能

#### 1.1 クラス階層情報の保存
**実装場所**: `pygen_types.py`の`MClass`クラス

```python
@dataclass
class MClass:
    name: str
    desc: str
    funcs: List[MFunction]
    props: List[MArgument]
    namespace: str = ""
    filename: str = ""
    parent_classes: List[str] = field(default_factory=list)  # 追加
    is_abstract: bool = False  # 追加
```

**実装場所**: `main.py`の`_parse_class`メソッド

```python
def _parse_class(self, node: CppHeaderParser.CppClass, nests: List[str]):
    # ... 既存コード ...
    
    # 親クラス情報の取得を追加
    c.parent_classes = node.get("inherits", [])
    c.is_abstract = "abstract" in node.get("declaration", "")
```

#### 1.2 クラスレジストリの生成
**実装場所**: 新規ファイル `pygen/ReflectionWriter.py`

```python
class ReflectionWriter:
    """クラスのリフレクション情報を生成するWriter"""
    
    def generate_class_registry(self, classes: List[MClass]) -> str:
        """全クラスの登録コードを生成
        
        例:
        ClassRegistry::Register<TextEntt>("TextEntt", "Entity");
        ClassRegistry::Register<Movie>("Movie", "Entity");
        """
```

**実装場所**: 新規C++ヘッダー `movutl/core/reflection.hpp`

```cpp
namespace mu {
    class ClassRegistry {
    public:
        template<typename T>
        static void Register(const char* name, const char* parent);
        
        static Entity* CreateByName(const char* name);
        static std::vector<std::string> GetAllClassNames();
        static std::vector<std::string> GetDerivedClasses(const char* base);
        static const char* GetParentClass(const char* name);
    };
}
```

---

## 2. プロパティの詳細なメタデータ

### 現状の問題
- プロパティのアクセス修飾子（public/protected/private）が記録されていない
- プロパティのフラグ（BlueprintReadOnly、EditAnywhere等）の種類が限定的
- 列挙型プロパティの値リストが取得できない
- 配列型プロパティへの対応が不十分

### 必要な機能

#### 2.1 拡張されたプロパティフラグ
**実装場所**: `movutl/core/props.hpp`の`PropInfoFlags`

現状のフラグ定義:
```cpp
enum PropInfoFlags : uint16_t {
    PInfo_ReadOnly = 1 << 1,
    PInfo_NotVisibleInspector = 1 << 2,
    PInfo_Angle = 1 << 3,
    PInfo_Radian = 1 << 4,
    PInfo_Slider = 1 << 5,
    PInfo_PositionProp = 1 << 6,
    PInfo_RotationProp = 1 << 7,
    PInfo_ScaleProp = 1 << 8,
    PInfo_BlueprintReadWrite = 1 << 9,
};
```

新規追加が必要なフラグ:
```cpp
enum PropInfoFlags : uint16_t {
    // ... 既存のフラグ ...
    
    PInfo_EditAnywhere = 1 << 10,          // インスペクターで編集可能
    PInfo_EditDefaultsOnly = 1 << 11,      // デフォルト値のみ編集可能
    PInfo_VisibleAnywhere = 1 << 12,       // 表示のみ
    PInfo_Transient = 1 << 13,             // シリアライズ対象外
    PInfo_Config = 1 << 14,                // 設定ファイルに保存
    PInfo_BlueprintReadOnly = 1 << 15,     // Blueprintから読み取り専用
};
```

#### 2.2 列挙型プロパティの対応強化
**実装場所**: `pygen_types.py`の`ArgumentType`

```python
class ArgumentType(Enum):
    # ... 既存の型 ...
    ArgType_Enum = 13  # 追加: 列挙型
```

**実装場所**: `utils.py`の`parse_mprop_info`

```python
def parse_mprop_info(arg: MArgument, line: str, enums: List[MEnum]) -> MArgument:
    """enumの型情報も渡して、列挙型かどうかを判定できるようにする"""
    # ... 既存コード ...
    
    # 列挙型の場合、その値リストを記録
    if arg.ptype == ArgumentType.ArgType_Enum:
        arg.enum_values = find_enum_values(arg.c_type, enums)
```

#### 2.3 配列・コンテナ型の対応
**実装場所**: `pygen_types.py`の`MArgument`クラス

現状の定義:
```python
@dataclass
class MArgument:
    name: str = ""
    dispname: str = ""
    c_type: str = ""
    ptype: ArgumentType = ArgumentType.ArgType_Undefined
    detault: str = ""  # NOTE: タイポあり（defaultが正しい）
    category: str = ""
    desc: str = ""
    is_angle: bool = False
    readonly: bool = False
    minvalue: str = ""
    maxvalue: str = ""
    namespace :str = ""  # NOTE: 不要なスペースあり
    step: str = ""
```

追加が必要なフィールド:
```python
    is_array: bool = False  # 追加
    container_type: str = ""  # 追加: "vector", "array", "map" など
    element_type: str = ""  # 追加: 配列の要素型
```

**実装場所**: `utils.py`に新規関数

```python
def parse_container_type(c_type: str) -> Tuple[bool, str, str]:
    """
    コンテナ型を解析
    
    Returns:
        (is_container, container_type, element_type)
        例: "std::vector<int>" -> (True, "vector", "int")
    """
```

---

## 3. 関数のメタデータ拡張

### 現状の問題
- 関数のカテゴリ分け（Blueprint、Native、Editor等）が不足
- 関数の呼び出し権限（Server、Client、Authority等）が記録されていない
- 戻り値のメタデータが不十分

### 必要な機能

#### 3.1 関数属性フラグの追加
**実装場所**: `pygen_types.py`の`MFunction`

```python
@dataclass
class MFunction:
    name: str
    returns: MArgument = field(default_factory=MArgument)
    args: List[MArgument] = field(default_factory=list)
    desc: str = ""
    should_autogen: bool = False
    is_static: bool = False
    is_const: bool = False
    constructor: bool = False
    destructor: bool = False
    namespace: str = ""
    filename: str = ""
    
    # 新規追加
    is_virtual: bool = False
    is_pure_virtual: bool = False
    is_blueprint_callable: bool = False  # Blueprintから呼び出し可能
    is_blueprint_pure: bool = False      # 副作用なし
    category: str = ""                   # 関数のカテゴリ
    keywords: List[str] = field(default_factory=list)  # 検索用キーワード
```

**実装場所**: `main.py`の`_parse_function`

```python
def _parse_function(self, v: CppHeaderParser.CppMethod) -> MFunction | None:
    # ... 既存コード ...
    
    # 仮想関数の判定を追加
    f.is_virtual = v.get("virtual", False)
    f.is_pure_virtual = "= 0" in v.get("declaration", "")
    
    # MFUNCマクロから追加情報を取得
    lines = self.basestr.split("\n")
    if len(lines) >= v["line_number"]:
        line = lines[v["line_number"] - 1]
        f = parse_mfunc_info(f, line)  # 新規関数
```

---

## 4. 自動シリアライズ機能の強化

### 現状の問題
- JSONのみ対応（バイナリ形式が未対応）
- カスタム型のシリアライズ方法を指定できない
- バージョン管理機能がない
- デルタシリアライズ（変更された値のみ保存）が未対応

### 必要な機能

#### 4.1 バイナリシリアライズの追加
**実装場所**: 新規ファイル `movutl/core/props/binary_serialize.cpp`

```cpp
namespace mu {
    class BinarySerializer {
    public:
        static std::vector<uint8_t> Serialize(const Props& props);
        static Props Deserialize(const std::vector<uint8_t>& data);
        
        // ストリーミング用
        static void SerializeToStream(std::ostream& stream, const Props& props);
        static Props DeserializeFromStream(std::istream& stream);
    };
}
```

**実装場所**: `PropsWriter.py`に関数追加

```python
def _write_binary_serialize(self, cls: MClass):
    """バイナリシリアライズ関数を生成"""
    self.autogen_text += f"""
std::vector<uint8_t> {cls.name}::SerializeBinary() const {{
    return BinarySerializer::Serialize(this->getProps());
}}

void {cls.name}::DeserializeBinary(const std::vector<uint8_t>& data) {{
    this->setProps(BinarySerializer::Deserialize(data));
}}
"""
```

#### 4.2 カスタムシリアライザのサポート
**実装場所**: `pygen_types.py`の`MArgument`

```python
@dataclass
class MArgument:
    # ... 既存フィールド ...
    custom_serializer: str = ""  # 追加: カスタムシリアライザ関数名
    serialize_as: str = ""       # 追加: 別の型として保存
```

**実装場所**: `utils.py`の`parse_mprop_info`

MPROPERTYマクロに新しい属性を追加:
```cpp
// 使用例
MPROPERTY(name="カスタム", serializer="MyCustomSerializer")
MyCustomType custom_data;
```

#### 4.3 バージョン管理機能
**実装場所**: 新規ファイル `movutl/core/props/versioning.hpp`

```cpp
namespace mu {
    struct SerializeVersion {
        uint16_t major = 1;
        uint16_t minor = 0;
        uint16_t patch = 0;
    };
    
    class VersionedSerializer {
    public:
        // バージョン情報付きでシリアライズ
        static std::string SerializeWithVersion(
            const Props& props, 
            const SerializeVersion& version
        );
        
        // 古いバージョンからのマイグレーション
        static Props DeserializeAndMigrate(
            const std::string& data,
            const SerializeVersion& current_version,
            std::function<Props(Props, SerializeVersion)> migrator
        );
    };
}
```

**実装場所**: `PropsWriter.py`にバージョン情報生成

各クラスに`GetSerializeVersion()`メソッドを自動生成:
```python
def _write_serialize_version(self, cls: MClass):
    self.autogen_text += f"""
SerializeVersion {cls.name}::GetSerializeVersion() const {{
    return SerializeVersion{{1, 0, 0}};
}}
"""
```

#### 4.4 デルタシリアライズ（差分保存）
**実装場所**: `movutl/core/props.hpp`に新規メソッド

```cpp
class Props {
    // ... 既存メソッド ...
    
    // 追加: 差分のみを含むPropsを作成
    Props GetDelta(const Props& base) const;
    
    // 追加: デフォルト値と異なる値のみを保存
    std::string dump_json_delta(const PropsInfo& info, int indent = 2) const;
};
```

**実装場所**: `PropsWriter.py`に関数追加

```python
def _write_serialize_delta(self, cls: MClass):
    """デルタシリアライズ関数を生成"""
    self.autogen_text += f"""
std::string {cls.name}::SerializeDelta() const {{
    auto default_props = this->getPropsInfo().get_default();
    return this->getProps().GetDelta(default_props).dump_json();
}}
"""
```

---

## 5. 自動UI生成の強化

### 現状の問題
- UI要素の詳細な制御ができない（ツールチップ、アイコン等）
- プロパティのグループ化が弱い
- 条件付き表示（他のプロパティの値による表示/非表示）が未対応
- カスタムUIウィジェットの指定ができない

### 必要な機能

#### 5.1 UI表示の詳細制御
**実装場所**: `pygen_types.py`の`MArgument`

追加が必要なフィールド（既存の定義は2.3を参照）:
```python
    tooltip: str = ""              # 追加: ツールチップテキスト
    icon: str = ""                 # 追加: アイコンのパス
    group: str = ""                # 追加: プロパティグループ
    order: int = 0                 # 追加: 表示順序
    widget_type: str = ""          # 追加: カスタムウィジェット名
    visible_condition: str = ""    # 追加: 表示条件式
```

**実装場所**: `utils.py`の`parse_mprop_info`でパース

MPROPERTYマクロの拡張例:
```cpp
// 使用例
MPROPERTY(
    name="透明度", 
    tooltip="0-255の範囲で指定", 
    group="表示設定",
    order=1,
    visible_if="with_alpha==true"
)
uint8_t alpha_;
```

#### 5.2 プロパティグループの自動生成
**実装場所**: `PropsWriter.py`に新規Writer

```python
class UIMetadataWriter:
    """UI表示用のメタデータを生成"""
    
    def generate_property_groups(self, cls: MClass) -> str:
        """プロパティをグループ化したUIメタデータを生成
        
        戻り値例:
        {
            "表示設定": ["alpha_", "visible"],
            "変形": ["pos", "scale", "rotation"]
        }
        """
```

**実装場所**: 新規C++ファイル `movutl/core/props/ui_metadata.hpp`

```cpp
namespace mu {
    struct PropertyGroup {
        std::string name;
        std::vector<std::string> properties;
        bool collapsed = false;  // デフォルトで折りたたむか
    };
    
    class UIMetadata {
    public:
        static std::vector<PropertyGroup> GetPropertyGroups(const char* class_name);
        static std::string GetTooltip(const char* class_name, const char* prop_name);
        static bool IsVisible(const Entity* entity, const char* prop_name);
    };
}
```

#### 5.3 カスタムUIウィジェットのサポート
**実装場所**: `movutl/core/props.hpp`の`PropInfoBase`

```cpp
struct PropInfoBase {
    // ... 既存フィールド ...
    
    // 追加
    std::string widget_type;  // "ColorPicker", "FilePicker", "Slider"等
    std::string widget_config; // JSON形式のウィジェット設定
};
```

---

## 6. Blueprintシステム的な機能

### 現状の問題
- ビジュアルスクリプティングのためのメタデータが不足
- イベント/デリゲート機能がない
- ノードベースエディタ用の情報が未対応

### 必要な機能

#### 6.1 イベント/デリゲートシステム
**実装場所**: 新規ファイル `movutl/core/delegate.hpp`

```cpp
namespace mu {
    // シンプルなデリゲート実装
    template<typename... Args>
    class Delegate {
    public:
        using FuncType = std::function<void(Args...)>;
        
        void Bind(FuncType func);
        void Unbind();
        void Execute(Args... args);
        bool IsBound() const;
    };
    
    // マルチキャストデリゲート
    template<typename... Args>
    class MulticastDelegate {
    public:
        using FuncType = std::function<void(Args...)>;
        
        void Add(FuncType func);
        void Remove(FuncType func);
        void Broadcast(Args... args);
    };
}
```

**実装場所**: `pygen_types.py`に新型追加

```python
@dataclass
class MDelegate:
    """デリゲート（イベント）情報"""
    name: str
    args: List[MArgument]
    is_multicast: bool = False
    desc: str = ""
```

**実装場所**: `main.py`にデリゲート解析を追加

```python
def _parse_delegates(self, node: CppHeaderParser.CppHeader):
    """MDELEGATE/MEVENT マクロを解析"""
    # MEVENTマクロからイベント情報を抽出
```

#### 6.2 ノードベースエディタ用メタデータ
**実装場所**: `pygen_types.py`の`MFunction`

```python
@dataclass
class MFunction:
    # ... 既存フィールド ...
    
    # Blueprint/ノードエディタ用
    node_color: str = ""           # ノードの色
    node_icon: str = ""            # ノードのアイコン
    compact_node: bool = False     # コンパクト表示
    is_latent: bool = False        # 非同期処理
    exec_pins: List[str] = field(default_factory=list)  # 実行ピン名
```

---

## 7. パフォーマンスと最適化

### 現状の問題
- リフレクション情報の取得が毎回計算される
- 大量のプロパティを持つクラスでの性能低下
- キャッシュ機構がない

### 必要な機能

#### 7.1 静的なリフレクション情報のキャッシュ
**実装場所**: 新規ファイル `pygen/StaticDataWriter.py`

```python
class StaticDataWriter:
    """コンパイル時に決定できるリフレクション情報を
    静的データとして生成"""
    
    def generate_static_metadata(self, classes: List[MClass]) -> str:
        """constexpr配列として生成
        
        例:
        constexpr PropertyMetadata TextEntt_Props[] = {
            {"pos_", PropT_Vec3, offsetof(TextEntt, pos_)},
            {"alpha_", PropT_Int, offsetof(TextEntt, alpha_)},
        };
        """
```

#### 7.2 プロパティアクセスの最適化
**実装場所**: `PropsWriter.py`に最適化版生成

```python
def _write_fast_property_access(self, cls: MClass):
    """オフセットベースの高速プロパティアクセスを生成
    
    switch-caseやハッシュマップでのプロパティアクセスを実装
    """
```

---

## 8. エディタ拡張機能

### 現状の問題
- エディタでの表示順序の制御が弱い
- プロパティの検証機能がない
- カスタムエディタの作成が困難

### 必要な機能

#### 8.1 プロパティバリデーション
**実装場所**: `pygen_types.py`の`MArgument`

追加が必要なフィールド（既存の定義は2.3を参照）:
```python
    validators: List[str] = field(default_factory=list)  # バリデータ関数名
    error_message: str = ""  # バリデーション失敗時のメッセージ
```

**実装場所**: 新規C++ファイル `movutl/core/props/validation.hpp`

```cpp
namespace mu {
    class PropertyValidator {
    public:
        using ValidatorFunc = std::function<bool(const Props::Value&)>;
        
        static void RegisterValidator(
            const char* class_name,
            const char* prop_name,
            ValidatorFunc validator,
            const char* error_msg
        );
        
        static bool Validate(
            const char* class_name,
            const Props& props,
            std::vector<std::string>& errors
        );
    };
}
```

#### 8.2 プロパティ変更通知
**実装場所**: `PropsWriter.py`のsetProps生成を拡張

```python
def _write_setProps_with_notification(self, cls: MClass):
    """プロパティ変更時にコールバックを呼ぶsetPropsを生成"""
    self.autogen_text += f"""
void {cls.name}::setProps(const Props& p) {{
    // 変更前の値を保存
    auto old_props = this->getProps();
    
    // 既存のsetPropsロジック
    // ...
    
    // 変更されたプロパティに対してコールバック実行
    for(const auto& [key, value] : p.values) {{
        if(!old_props.contains(key) || old_props[key] != value) {{
            this->OnPropertyChanged(key, old_props.get_(key), value);
        }}
    }}
}}
"""
```

---

## 実装優先順位

実装の優先順位を以下のように提案します：

### Phase 1: 基礎機能の強化（重要度：高）
1. **クラス階層のリフレクション** (セクション1)
   - プロジェクト全体の基盤となる機能
   - 他の機能の前提条件

2. **プロパティメタデータの拡張** (セクション2.1, 2.2)
   - より詳細な型情報が必要
   - UI生成とシリアライズの改善に必須

3. **バイナリシリアライズ** (セクション4.1)
   - パフォーマンス向上に直結
   - 実用性が高い

### Phase 2: ユーザビリティ向上（重要度：中）
4. **UI生成の強化** (セクション5.1, 5.2)
   - エディタ体験の向上
   - 開発効率に影響

5. **関数メタデータの拡張** (セクション3)
   - スクリプティング機能の強化

6. **バージョン管理とデルタ保存** (セクション4.3, 4.4)
   - 長期的なメンテナンス性

### Phase 3: 高度な機能（重要度：低）
7. **イベント/デリゲート** (セクション6.1)
   - アーキテクチャの変更を伴う

8. **パフォーマンス最適化** (セクション7)
   - 必要になってから実装

9. **エディタ拡張** (セクション8)
   - 特定のユースケース向け

---

## まとめ

現状のpygenは基本的なプロパティ管理とLuaバインディング生成には対応していますが、UPROPERTYレベルの機能を実現するには以下が必要です：

### 最も重要な追加要素
1. **クラスのリフレクション情報** - 動的なクラス生成、階層情報
2. **拡張されたメタデータ** - より詳細なプロパティ・関数情報
3. **バイナリシリアライズ** - パフォーマンス重視のシリアライズ
4. **UI生成の強化** - グループ化、条件付き表示、カスタムウィジェット

### ファイル構成の提案
```
pygen/
├── main.py              # 既存: メインパーサー
├── pygen_types.py       # 拡張: 新しいフィールド追加
├── PropsWriter.py       # 拡張: 新機能追加
├── ReflectionWriter.py  # 新規: リフレクション情報生成
├── StaticDataWriter.py  # 新規: 静的メタデータ生成
└── UIMetadataWriter.py  # 新規: UI情報生成

movutl/core/
├── props.hpp            # 拡張: 新しいメソッド追加
├── reflection.hpp       # 新規: リフレクションAPI
├── props/
│   ├── json_loadsave.cpp      # 既存
│   ├── binary_serialize.cpp   # 新規: バイナリ形式
│   ├── versioning.hpp         # 新規: バージョン管理
│   ├── validation.hpp         # 新規: バリデーション
│   └── ui_metadata.hpp        # 新規: UI情報

movutl/core/
└── delegate.hpp         # 新規: イベント/デリゲート
```

このドキュメントが実装の指針となれば幸いです。
