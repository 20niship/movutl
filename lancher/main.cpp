#include <chrono>
#include <movutl/app/app.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/render2d/render2d.hpp>
#include <thread>

using namespace mu;

void util_show_main_cmp_result() {
  auto cmp = Project::Get()->get_main_comp();
  MU_ASSERT(cmp);
  render_comp(cmp);
  auto frame = cmp->frame_final;
  MU_ASSERT(frame);
}

void create_sample_video_data() {
  Project::New();
  add_new_video_track("testname", "../assets/movies/big_buck_bunny_360_10s.mp4", 30, 0);
}

int main(int argc, char** argv) {
  mu::init();
  if(argc > 1) {
    mu::run_lua_file(argv[1]);
  } else {
    create_sample_video_data();
    util_show_main_cmp_result();
  }
  while(!mu::should_terminate()) {
    mu::update();
  }
  mu::terminate();
}
