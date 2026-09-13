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
    cmp->frame = std::clamp(cmp->frame.load() + dir_, cmp->fstart, cmp->fend);
    return CommandStatus::Finished;
  }

  int dir_;
};

// entt->fstart_/fend_の範囲からエンティティをComposition::layersから取り除く(Compositionにremove_entity APIが無いため直接操作)
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
    int orig_fend;        // 分割前のoriginal->fend_
  };

  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    int frame = cmp->frame;

    for(auto& entt : get_selected_entts()) {
      if(!entt) continue;
      if(entt->fstart_ >= frame || frame >= entt->fend_) continue; // 現在フレームがクリップ範囲外

      auto clone = duplicate_asset(entt);
      if(!clone) {
        LOG_F(WARNING, "SplitCommand: unsupported entity type for '%s', skipping", entt->name.c_str());
        continue;
      }

      SplitState st;
      st.original  = entt;
      st.orig_fend = entt->fend_;

      // fstart_/fend_/anchor_等はduplicate_asset内のgetSaveProps/fromSavePropsで既にコピー済み
      clone->fstart_ = frame; // 後半
      entt->fend_    = frame; // 前半

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
      st.original->fend_ = st.orig_fend;
      if(st.clone) remove_entity_from_comp(st.clone->get_comp(), st.clone);
    }
  }

  // 分割を再適用する: 元クリップのfendを縮め、複製クリップを再度挿入する
  void on_redo() override {
    for(auto& st : splits_) {
      if(!st.original || !st.clone) continue;
      st.original->fend_ = st.clone->fstart_;
      auto* comp         = st.original->get_comp();
      if(comp) comp->insert_entity(st.clone, -1);
    }
  }

  bool undoable() const override { return true; }

  std::vector<SplitState> splits_;
};

// 選択中Entityの現在フレームの中間点(anim_props_本体+全filters_のパラメータ横断)を一括トグルする
struct ToggleKeyframeCommand final : mCommand {
  struct EntityState {
    Ref<Entity> entt;
    cutil::Prop before_anim, after_anim;
    std::vector<cutil::Prop> before_filter_anim, after_filter_anim;
  };

  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    uint32_t frame = (uint32_t)std::max(cmp->frame.load(), 0);

    states_.clear();
    for(auto& entt : get_selected_entts()) {
      if(!entt) continue;
      entt->ensure_anim_props();

      EntityState st;
      st.entt        = entt;
      st.before_anim = entt->anim_props_.save();
      for(auto& f : entt->filters_) st.before_filter_anim.push_back(f.props.save());

      bool has_any = false;
      for(int i = 0; i < (int)entt->anim_props_.props.size() && !has_any; i++)
        if(entt->anim_props_.has_key_at(i, frame)) has_any = true;
      for(auto& f : entt->filters_) {
        if(has_any) break;
        for(int i = 0; i < (int)f.props.props.size() && !has_any; i++)
          if(f.props.has_key_at(i, frame)) has_any = true;
      }

      if(has_any) {
        entt->erase_keyframes_at(frame);
      } else {
        for(int i = 0; i < (int)entt->anim_props_.props.size(); i++) entt->anim_props_.add_keyframe_here(i, frame);
        for(auto& f : entt->filters_)
          for(int i = 0; i < (int)f.props.props.size(); i++) f.props.add_keyframe_here(i, frame);
      }

      st.after_anim = entt->anim_props_.save();
      for(auto& f : entt->filters_) st.after_filter_anim.push_back(f.props.save());

      if(auto* comp = entt->get_comp()) comp->invalidate_cache_range(entt->fstart_, entt->fend_);
      states_.push_back(std::move(st));
    }
    return CommandStatus::Finished;
  }

  void on_undo() override {
    for(auto& st : states_) {
      if(!st.entt) continue;
      st.entt->anim_props_.load_keys(st.before_anim);
      for(size_t i = 0; i < st.entt->filters_.size() && i < st.before_filter_anim.size(); i++) st.entt->filters_[i].props.load_keys(st.before_filter_anim[i]);
      if(auto* comp = st.entt->get_comp()) comp->invalidate_cache_range(st.entt->fstart_, st.entt->fend_);
    }
  }

  void on_redo() override {
    for(auto& st : states_) {
      if(!st.entt) continue;
      st.entt->anim_props_.load_keys(st.after_anim);
      for(size_t i = 0; i < st.entt->filters_.size() && i < st.after_filter_anim.size(); i++) st.entt->filters_[i].props.load_keys(st.after_filter_anim[i]);
      if(auto* comp = st.entt->get_comp()) comp->invalidate_cache_range(st.entt->fstart_, st.entt->fend_);
    }
  }

  bool undoable() const override { return true; }

  std::vector<EntityState> states_;
};

// undo/redoコマンド自身はCommandManagerの履歴には積まない(undoable()==false、既定のまま)
struct UndoCommand final : mCommand {
  CommandStatus on_start() override { return undo_command() ? CommandStatus::Finished : CommandStatus::Failed; }
};

struct RedoCommand final : mCommand {
  CommandStatus on_start() override { return redo_command() ? CommandStatus::Finished : CommandStatus::Failed; }
};

} // namespace

void register_default_commands() {
  register_command<PlayPauseCommand>({"play_pause", "再生/一時停止", "コンポジションの再生/一時停止をトグルする", "space"});
  register_command<FrameStepCommand>({"frame_step_forward", "次のフレーム", "現在フレームを1つ進める", "right"}, 1);
  register_command<FrameStepCommand>({"frame_step_backward", "前のフレーム", "現在フレームを1つ戻す", "left"}, -1);
  register_command<SplitCommand>({"split", "分割", "選択中のクリップを現在フレームで分割する", "s"});
  register_command<ToggleKeyframeCommand>({"toggle_keyframe", "中間点をトグル", "選択中オブジェクトの現在フレームの中間点を追加/削除する", "p"});
  register_command<UndoCommand>({"undo", "元に戻す", "直前の操作を取り消す", "ctrl+z"});
  register_command<RedoCommand>({"redo", "やり直し", "取り消した操作をやり直す", "ctrl+y"});
}

} // namespace mu::detail
