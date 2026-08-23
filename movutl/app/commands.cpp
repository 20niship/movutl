#include <algorithm>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/time.hpp>

namespace mu::detail {

namespace {

struct PlayPauseCommand final : mCommand {
  CommandStatus on_start() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp) return CommandStatus::Failed;
    s_playing = !s_playing;
    if(!s_playing) return CommandStatus::Finished;
    last_time_ = mu_now_seconds();
    return CommandStatus::Running;
  }

  CommandStatus tick() override {
    auto cmp = Composition::GetActiveComp();
    if(!cmp || !s_playing) return CommandStatus::Finished;

    double now = mu_now_seconds();
    double fps = cmp->framerate_de != 0 ? (double)cmp->framerate_nu / cmp->framerate_de : 30.0;
    if(now - last_time_ >= 1.0 / fps) {
      cmp->frame++;
      if(cmp->frame > cmp->fend) cmp->frame = cmp->fstart;
      last_time_ = now;
    }
    return CommandStatus::Running;
  }

  void on_cancel() override { s_playing = false; }

  double last_time_ = 0;
  static bool s_playing; // run_command()の度に新しいインスタンスが作られるため、トグル状態はインスタンスをまたいでstaticで共有する
};
bool PlayPauseCommand::s_playing = false;

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

} // namespace

void register_default_commands() {
  register_command<PlayPauseCommand>({"play_pause", "再生/一時停止", "コンポジションの再生/一時停止をトグルする", "space"});
  register_command<FrameStepCommand>({"frame_step_forward", "次のフレーム", "現在フレームを1つ進める", "right"}, 1);
  register_command<FrameStepCommand>({"frame_step_backward", "前のフレーム", "現在フレームを1つ戻す", "left"}, -1);
}

} // namespace mu::detail
