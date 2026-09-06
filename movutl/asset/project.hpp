#pragma once

#include <cutil/ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/vector.hpp>
#include <string>

namespace mu {

class Project {
public:
  Project()  = default;
  ~Project() = default;
  MOVUTL_DECLARE_SINGLETON(Project);

  std::string path;
  std::string output_path; //	出力ファイル名 (ファイル名が決まっていない時は何も入っていません)

  std::vector<Ref<Entity>> entities;
  std::vector<Ref<Composition>> compos_; // vectorが所有権を持つ(ヒープ確保のためpush_backでの再配置後もCompositionへの生ポインタは有効)
  int main_comp_idx = 0;

  static void New(int width = 1920, int height = 1080, int fps = 30);
  static void Save(const char* path = nullptr);
  static void Load(const char* path);

  // Compositionを新規追加し、設定ウィンドウを自動で開いた状態で返す(戻り値は非所有ポインタ、所有権はcompos_が持つ)
  static Composition* AddComposition(const char* name, int width = 1920, int height = 1080, int fps = 30);

  [[deprecated]] Composition* get_main_comp() {
    if(main_comp_idx < 0 || main_comp_idx >= compos_.size()) {
      Project::New();
      return compos_[main_comp_idx].get();
    }
    return compos_[main_comp_idx].get();
  }

  static Composition* GetActiveCompo();
  static void SetActiveCompo(int idx);
};

} // namespace mu
