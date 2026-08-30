#include <algorithm>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/logger.hpp>

namespace mu::detail {

namespace {

// 再生/停止の実体はAppMain側(app.cpp)が持つ。このコマンドは薄いトグルラッパー
struct PlayPauseCommand final : mCommand {
  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    if(is_playing())
      pause();
    else
      play();
    return CommandStatus::Finished;
  }
};

struct FrameStepCommand final : mCommand {
  explicit FrameStepCommand(int dir) : dir_(dir) {}

  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    std::lock_guard<std::mutex> lock(cmp->mtx);
    cmp->frame = std::clamp(cmp->frame + dir_, cmp->fstart, cmp->fend);
    return CommandStatus::Finished;
  }

  int dir_;
};

// entt->trk範囲からエンティティをComposition::layersから取り除く(Compositionにremove_entity APIが無いため直接操作)
void remove_entity_from_comp(Composition* comp, const Ref<Entity>& entt) {
  if(!comp || !entt) return;
  for(auto& layer : comp->layers) {
    auto& v = layer.entts;
    v.erase(std::remove(v.begin(), v.end(), entt), v.end());
  }
}

struct SplitCommand final : mCommand {
  // 1クリップの分割前後の状態。on_undo/on_redoで使う
  struct SplitState {
    Ref<Entity> original; // 分割された元のクリップ(前半)
    Ref<Entity> clone;    // 分割で新規生成されたクリップ(後半)
    int orig_fend;        // 分割前のoriginal->trk.fend
  };

  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    int frame = cmp->frame;

    for(auto& entt : get_selected_entts()) {
      if(!entt) continue;
      if(entt->trk.fstart >= frame || frame >= entt->trk.fend) continue; // 現在フレームがクリップ範囲外

      auto clone = duplicate_asset(entt);
      if(!clone) {
        LOG_F(WARNING, "SplitCommand: unsupported entity type for '%s', skipping", entt->name.c_str());
        continue;
      }

      SplitState st;
      st.original  = entt;
      st.orig_fend = entt->trk.fend;

      clone->trk        = entt->trk; // fstart/fend/anchor等をコピー
      clone->trk.fstart = frame;     // 後半
      entt->trk.fend    = frame;     // 前半

      auto* comp = entt->get_comp();
      if(comp) comp->insert_entity(clone, -1);

      st.clone = clone;
      splits_.push_back(std::move(st));
    }
    return CommandStatus::Finished;
  }

  // 分割前の状態に戻す: 複製クリップを削除し、元クリップのfendを復元する
  void on_undo() override {
    for(auto& st : splits_) {
      if(!st.original) continue;
      st.original->trk.fend = st.orig_fend;
      if(st.clone) remove_entity_from_comp(st.clone->get_comp(), st.clone);
    }
  }

  // 分割を再適用する: 元クリップのfendを縮め、複製クリップを再度挿入する
  void on_redo() override {
    for(auto& st : splits_) {
      if(!st.original || !st.clone) continue;
      st.original->trk.fend = st.clone->trk.fstart;
      auto* comp            = st.original->get_comp();
      if(comp) comp->insert_entity(st.clone, -1);
    }
  }

  std::vector<SplitState> splits_;
};

} // namespace

void register_default_commands() {
  register_command<PlayPauseCommand>({"play_pause", "再生/一時停止", "コンポジションの再生/一時停止をトグルする", "space"});
  register_command<FrameStepCommand>({"frame_step_forward", "次のフレーム", "現在フレームを1つ進める", "right"}, 1);
  register_command<FrameStepCommand>({"frame_step_backward", "前のフレーム", "現在フレームを1つ戻す", "left"}, -1);
  register_command<SplitCommand>({"split", "分割", "選択中のクリップを現在フレームで分割する", "s"});
}

} // namespace mu::detail
