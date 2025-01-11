#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
//
#include <movutl/asset/movie.hpp>
#include <movutl/plugin/input.hpp>

namespace mu::detail {

static bool fn_init() {}
static bool fn_exit() {}

struct InHandleCVVideo {
  int frame;
  cv::VideoCapture cap;
  InHandleCVVideo(const char* file) : frame(0), cap(file) {}
};

static INPUT_HANDLE fn_open(const char* file) {
  InHandleCVVideo* ih = new InHandleCVVideo(file);
  ih->frame = 0;
  if(!ih->cap.isOpened()) {
    delete ih;
    return nullptr;
  }
  return new cv::VideoCapture(ih->cap);
}

static bool fn_close(INPUT_HANDLE ih) {
  if(!ih) return false;
  auto cap = (InHandleCVVideo*)ih;
  if(cap) delete cap;
  return true;
}

static bool fn_info_get(INPUT_HANDLE ih, InputInfo* iip) {
  if(!ih) return false;
  MU_ASSERT(iip);
  cv::VideoCapture* cap = (cv::VideoCapture*)ih;
  if(!cap) return false;
  iip->flag = InputFlag::Video;
  iip->framerate = cap->get(cv::CAP_PROP_FPS);
  iip->nframes = cap->get(cv::CAP_PROP_FRAME_COUNT);
  iip->size[0] = cap->get(cv::CAP_PROP_FRAME_WIDTH);
  iip->size[1] = cap->get(cv::CAP_PROP_FRAME_HEIGHT);
  iip->handler = 0;
  return true;
}

int fn_read_video(INPUT_HANDLE ih, const InputInfo* iip, Movie* entity) {
  if(!ih) return 0;
  cv::VideoCapture* cap = (cv::VideoCapture*)ih;
  if(!cap) return 0;
  cv::Mat image;
  *cap >> image;
  if(image.empty()) return 0;
  entity->img_->set_cv_img(&image);
  return image.total() * image.elemSize();
}

static int fn_set_frame(INPUT_HANDLE ih, int frame) {
  if(!ih) return 0;
  auto cap = (InHandleCVVideo*)ih;
  cap->frame = frame;
  cap->cap.set(cv::CAP_PROP_POS_FRAMES, frame);
  return frame;
}

static int fn_get_frame(INPUT_HANDLE ih) {
  if(!ih) return 0;
  InHandleCVVideo* cap = (InHandleCVVideo*)ih;
  if(!cap) return 0;
  return cap->frame;
}

mu::InputPluginTable plg_video_reader = {
  0x00000001,            // guid
  InputFlag::Video,      // supports
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
