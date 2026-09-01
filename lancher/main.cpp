#include <chrono>
#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/plugin/input.hpp>
#include <thread>

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

// EntityType_Movie対応プラグインは拡張子だけで判定するため音声専用ファイルも「対応」を返す。開いて実際に確認する
bool has_video_stream(const char* path) {
  auto* plg = get_compatible_plugin(path, EntityType_Movie);
  if(!plg) return false;
  InputHandle h = plg->fn_open(path);
  if(!h) return false;
  EntityInfo info;
  bool ok = plg->fn_info_get && plg->fn_info_get(h, &info) && info.width > 0 && info.height > 0 && info.nframes > 0;
  plg->fn_close(h);
  return ok;
}

// 動画/音声/画像のいずれかを内容から自動判別してトラックへ開く
void open_media_file(const char* path) {
  Project::New();
  if(has_video_stream(path)) {
    add_new_video_track("media", path, 0, 0);
  } else if(get_compatible_plugin(path, EntityType_Audio)) {
    add_new_audio_track("media", path, 0, 0);
  } else if(get_compatible_plugin(path, EntityType_Image)) {
    auto img = Image::Create("media", path);
    if(img->width == 0) {
      LOG_F(ERROR, "Failed to open image file: %s", path);
      return;
    }
    Composition* main_comp = Composition::GetActiveComp();
    img->fstart            = 0;
    img->fend              = Config::Get()->default_image_frames;
    main_comp->insert_entity(img);
  } else {
    LOG_F(ERROR, "No compatible plugin found for file: %s", path);
  }
}

int main(int argc, char** argv) {
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
