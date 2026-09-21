#include <chrono>
#include <cstring>
#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/render2d/renderer_registry.hpp>
#include <thread>
#include <vector>

using namespace mu;

void util_show_main_cmp_result() {
  auto cmp = Project::Get()->get_main_comp();
  MU_ASSERT(cmp);
  auto frame = cmp->render_current_frame_main_thread();
  MU_ASSERT(frame);
}

void create_sample_video_data() {
  Project::New();
  add_new_video_track("testname", "../assets/movies/big_buck_bunny_360_10s.mp4", 30, 0);
}

// 動画/音声/画像のいずれかを内容から自動判別してトラックへ開く
void open_media_file(const char* path) {
  Project::New();
  open_file(path); // D&D/メニューと同じ経路(拡張子でexo/プロジェクト/メディアを判別)
}

int main(int argc, char** argv) {
  // --renderer=<name> は位置引数(ファイル)と区別して取り除く
  std::vector<char*> args{argv[0]};
  for(int i = 1; i < argc; i++) {
    if(std::strncmp(argv[i], "--renderer=", 11) == 0)
      set_renderer_cli_override(argv[i] + 11);
    else
      args.push_back(argv[i]);
  }
  argc = (int)args.size();
  argv = args.data();
  mu::init();
  if(argc > 1) {
    if(fs_extension(argv[1]) == "lua") {
      mu::run_lua_file(argv[1]);
    } else {
      open_media_file(argv[1]);
      util_show_main_cmp_result();
    }
  } else {
    create_sample_video_data();
    util_show_main_cmp_result();
  }
  while(!mu::should_terminate()) {
    mu::update();
  }
  mu::terminate();
}
