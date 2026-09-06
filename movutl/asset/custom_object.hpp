#pragma once

#include <cutil/prop.hpp>
#include <functional>
#include <memory>
#include <movutl/asset/entity.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <string>
#include <unordered_map>
#include <vector>

extern "C" {
struct lua_State;
}

namespace mu {

// Luaで定義されたカスタムオブジェクト1個のインスタンス。1つのdefにつき複数インスタンスが存在しうる
class CustomObjectEntt final : public Entity {
private:
  const detail::AviUtlScriptDef* def_ = nullptr; // CustomObjectRegistryが所有する定義への非所有ポインタ(寿命はアプリ終了まで)
  lua_State* L_                       = nullptr;
  int body_ref_                       = -2;        // LUA_NOREFと同値。lua.hをヘッダへ持ち込まないためintで保持し、.cppで初期化する
  std::unordered_map<std::string, Image> buffers_; // obj.copybuffer用(フィルタ版のAviUtlFilterStateと同じ役割)

public:
  std::string script_name_; // どの定義を使っているか(保存/復元用)
  cutil::Prop params_;      // track0-3 / check0-3 相当の現在値

  CustomObjectEntt() = default;
  ~CustomObjectEntt() override;

  static Ref<CustomObjectEntt> Create(const char* name, const std::string& script_name);

  EntityType getType() const override { return EntityType_Custom; }
  bool render(Composition* cmp, Image* target, int frame) override;

  // インスペクタでtrack0-3/check0-3相当のパラメータを編集するためのUI向けアクセサ(nullptrはスクリプト未解決を意味する)
  const detail::AviUtlScriptDef* def() const { return def_; }

  cutil::Prop getProps() const override;
  void setProps(const cutil::Prop& props) override;
};

// カスタムオブジェクト定義の登録簿。名前→(定義, インスタンス生成factory)
class CustomObjectRegistry {
  MOVUTL_DECLARE_SINGLETON(CustomObjectRegistry);

public:
  struct Entry {
    std::string name;
    std::function<Ref<Entity>()> factory; // 名前+開始位置未設定のCustomObjectEnttを1個生成する
  };

  CustomObjectRegistry()  = default;
  ~CustomObjectRegistry() = default;

  // defの所有権を受け取り、対応するfactoryを1件登録する
  void register_object(std::unique_ptr<detail::AviUtlScriptDef> def);

  const std::vector<Entry>& list() const { return entries_; }
  const detail::AviUtlScriptDef* find_def(const std::string& name) const;

private:
  std::vector<std::unique_ptr<detail::AviUtlScriptDef>> defs_; // 安定アドレスのための所有ストレージ
  std::vector<Entry> entries_;
};

} // namespace mu
