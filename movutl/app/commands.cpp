#include <algorithm>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/core/command.hpp>

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
    cmp->frame = std::clamp(cmp->frame + dir_, cmp->fstart, cmp->fend);
    return CommandStatus::Finished;
  }

  int dir_;
};

// 汎用clone機構が無いためMovie/Imageのみ対応、対応外の型は黙ってスキップする(ログのみ、Failedにはしない)。
struct SplitCommand final : mCommand {
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

} // namespace

void register_default_commands() {
  register_command<PlayPauseCommand>({"play_pause", "再生/一時停止", "コンポジションの再生/一時停止をトグルする", "space"});
  register_command<FrameStepCommand>({"frame_step_forward", "次のフレーム", "現在フレームを1つ進める", "right"}, 1);
  register_command<FrameStepCommand>({"frame_step_backward", "前のフレーム", "現在フレームを1つ戻す", "left"}, -1);
  register_command<SplitCommand>({"split", "分割", "選択中のクリップを現在フレームで分割する", "s"});
}

} // namespace mu::detail
