#pragma once
#include <imgui.h>
#include <map>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/audio/audio_player.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/plugin/plugin.hpp>
#include <movutl/render2d/render_worker.hpp>

namespace mu::detail {

class AppMain {
public:
  MOVUTL_DECLARE_SINGLETON(AppMain);
  AppMain()  = default;
  ~AppMain() = default;

  std::vector<FilterPluginTable> filters;
  std::vector<InputPluginTable> input_plugins;
  std::vector<OutputPluginTable> output_plugins;
  std::vector<PluginData> plugins;
  std::vector<Ref<Entity>> entt_selected;

  std::map<std::string, ImGuiStyle> imgui_styles;
  std::map<std::string, Workspace> workspaces;

  RenderWorkerPool render_pool;
  AudioMixWorker audio_worker;
  AudioPlayer audio_player;

  // ------------ 再生制御 ------------
  void play();
  void pause();
  void reset();
  void goto_frame(int frame);
  bool is_playing() const { return playing_; }

  void update_frame_impl(); // 毎フレーム呼ばれる。playing_中は経過時間とfpsから現在フレームを進める

private:
  bool playing_           = false;
  double last_frame_time_ = 0;
};

void register_default_commands();

} // namespace mu::detail
