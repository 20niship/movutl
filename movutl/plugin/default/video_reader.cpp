#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
//
#include <movutl/asset/movie.hpp>
#include <movutl/plugin/input.hpp>

namespace mu::detail {

static bool fn_init() {
  printf("OpenCV Video Reader initialized\n");
  return true;
}
static bool fn_exit() {
  return true;
}

struct InHandleCVVideo {
  int frame;
  cv::VideoCapture cap;
  InHandleCVVideo(const char* file) : frame(0), cap(file) {}
};

static InputHandle fn_open(const char* file) {
  InHandleCVVideo* ih = new InHandleCVVideo(file);
  ih->frame = 0;
  if(!ih->cap.isOpened()) {
    delete ih;
    return nullptr;
  }
  return ih;
}

static bool fn_close(InputHandle ih) {
  if(!ih) return false;
  auto cap = (InHandleCVVideo*)ih;
  if(cap) delete cap;
  return true;
}

static bool fn_info_get(InputHandle ih, EntityInfo* iip) {
  if(!ih) return false;
  MU_ASSERT(iip);
  auto c = (InHandleCVVideo*)ih;
  if(!c->cap.isOpened()) return false;
  iip->flag = EntityType_Movie;
  iip->framerate = c->cap.get(cv::CAP_PROP_FPS);
  iip->nframes = c->cap.get(cv::CAP_PROP_FRAME_COUNT);
  iip->width = c->cap.get(cv::CAP_PROP_FRAME_WIDTH);
  iip->height = c->cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  iip->handler = 0;
  return true;
}

int fn_read_video(InputHandle ih, const EntityInfo* iip, Movie* entity) {
  if(!ih) return 0;
  InHandleCVVideo* cap = (InHandleCVVideo*)ih;
  if(!cap->cap.isOpened()) return 0;
  cv::Mat image;
  cap->cap >> image;
  cv::imshow("imageaaaa", image);

  if(image.empty()) return 0;
  entity->img_->set_cv_img(&image);
  return image.total() * image.elemSize();
}

static int fn_set_frame(InputHandle ih, int frame) {
  if(!ih) return 0;
  auto cap = (InHandleCVVideo*)ih;
  cap->frame = frame;
  cap->cap.set(cv::CAP_PROP_POS_FRAMES, frame);
  return frame;
}

static int fn_get_frame(InputHandle ih) {
  if(!ih) return 0;
  InHandleCVVideo* cap = (InHandleCVVideo*)ih;
  if(!cap) return 0;
  return cap->frame;
}

mu::InputPluginTable plg_video_reader = {
  0x00000001,            // guid
  EntityType_Movie,      // supports
  "OpenCV Video Reader", // name
  "",                    // filepath
  "OpenCV Reader",       // information
  {"avi", "mp4", "mov", "mkv", "webm", "mpg", "mpeg", "m4v", "", ""},
  fn_init,       //	DLL開始時に呼ばれる関数へのポンタ (NULLなら呼ばれません)
  fn_exit,       //	DLL終了時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  fn_open,       //	入力ファイルをオープンする関数へのポインタ
  fn_close,      //	入力ファイルをクローズする関数へのポインタ
  fn_info_get,   //	入力ファイルの情報を取得する関数へのポインタ
  fn_set_frame,  //	フレームを設定する関数へのポインタ
  fn_get_frame,  //	現在のフレーム番号を取得する関数へのポインタ
  fn_read_video, //	画像データを読み込む関数へのポインタ
  nullptr,       //	音声データを読み込む関数へのポインタ
  nullptr,       //	キーフレームか調べる関数へのポインタ (NULLなら全てキーフレーム)
  nullptr,       //	入力設定のダイアログを要求された時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
};

} // namespace mu::detail
