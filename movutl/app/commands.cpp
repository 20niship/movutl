#include <algorithm>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/time.hpp>

namespace mu::detail {

struct PlayPauseCommand : mCommand {
  PlayPauseCommand() {
    id          = "play_pause";
    name        = "再生/一時停止";
    description = "コンポジションの再生/一時停止をトグルする";
    shortcut    = ImGuiKey_Space;
  }

  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    playing_   = !playing_;
    last_time_ = mu_now_seconds();
    return playing_ ? CommandStatus::Running : CommandStatus::Finished;
  }

  CommandStatus tick() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp || !playing_) return CommandStatus::Finished;

    double now = mu_now_seconds();
    double fps = cmp->framerate_de != 0 ? (double)cmp->framerate_nu / cmp->framerate_de : 30.0;
    if(now - last_time_ >= 1.0 / fps) {
      cmp->frame++;
      if(cmp->frame > cmp->fend) cmp->frame = cmp->fstart;
      last_time_ = now;
    }
    return CommandStatus::Running;
  }

  void on_cancel() override { playing_ = false; }

  bool playing_    = false;
  double last_time_ = 0;
};

struct FrameStepCommand : mCommand {
  explicit FrameStepCommand(int dir) : dir_(dir) {
    id          = dir_ > 0 ? "frame_step_forward" : "frame_step_backward";
    name        = dir_ > 0 ? "次のフレーム" : "前のフレーム";
    description = "現在フレームを1つ進める/戻す";
    shortcut    = dir_ > 0 ? ImGuiKey_RightArrow : ImGuiKey_LeftArrow;
  }

  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    cmp->frame = std::clamp(cmp->frame + dir_, cmp->fstart, cmp->fend);
    return CommandStatus::Finished;
  }

  int dir_;
};

// 汎用clone機構が無いためMovie/Imageのみ対応、対応外の型は黙ってスキップする(ログのみ、Failedにはしない)。
struct SplitCommand : mCommand {
  SplitCommand() {
    id          = "split";
    name        = "分割";
    description = "選択中のクリップを現在フレームで分割する";
    shortcut    = ImGuiKey_S;
  }

  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    int frame = cmp->frame;

    for(auto& entt : get_selected_entts()) {
      if(!entt) continue;
      if(entt->trk.fstart >= frame || frame >= entt->trk.fend) continue; // 現在フレームがクリップ範囲外

      Ref<Entity> clone;
      if(entt->getType() == EntityType_Movie) {
        auto* mov = static_cast<Movie*>(entt.get());
        clone     = Movie::Create(mov->name.c_str(), mov->path_.c_str());
      } else if(entt->getType() == EntityType_Image) {
        auto* img = static_cast<Image*>(entt.get());
        clone     = Image::Create(img->name.c_str(), img->path.c_str());
      } else {
        LOG_F(WARNING, "SplitCommand: unsupported entity type for '%s', skipping", entt->name.c_str());
        continue;
      }
      if(!clone) continue;

      clone->trk        = entt->trk; // fstart/fend/anchor等をコピー
      clone->trk.fstart = frame;     // 後半
      entt->trk.fend    = frame;     // 前半

      auto* comp = entt->get_comp();
      if(comp) comp->insert_entity(clone, -1);
    }
    return CommandStatus::Finished;
  }
};

void register_default_commands() {
  auto cm = CommandManager::Get();
  cm->register_command(cutil::make_ref<PlayPauseCommand>());
  cm->register_command(cutil::make_ref<FrameStepCommand>(1));
  cm->register_command(cutil::make_ref<FrameStepCommand>(-1));
  cm->register_command(cutil::make_ref<SplitCommand>());
}

} // namespace mu::detail
