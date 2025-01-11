#pragma once
#include <cstdint>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/array.hpp>
#include <movutl/core/defines.hpp>

#define BITMAPINFOHEADER void
#define WAVEFORMATEX void

namespace mu {
enum class InputFlag : uint16_t {
  Video = 1 << 0,
  Audio = 1 << 1,
  VideoRandomAccess = 1 << 3,
};
MOVUTL_DEFINE_ENUM_ATTR_BITFLAGS(InputFlag);

struct InputInfo {
  InputFlag flag = InputFlag::Video;
  float framerate = 23.98;    // フレームレート
  int32_t nframes = 0;        // フレーム数
  int32_t format_size;        // 画像フォーマッのサイズ
  Vec2d size;                 // 画像サイズ
  int32_t audio_n;            // 音声サンプル数
  WAVEFORMATEX* audio_format; // 音声フォーマットへのポインタ(次に関数が呼ばれるまで内容を有効にしておく)
  int32_t audio_format_size;  // 音声フォーマットのサイズ
  void* handler;              // 画像codecハンドラ
  int32_t reserve[7];
};

// プラグインがそのファイルを開いた時のインスタンスを返すときのポインタ
typedef void* INPUT_HANDLE;

#define MAX_NAME 64

//	入力プラグイン構造体
struct InputPluginTable {
  uint64_t guid;
  InputFlag supports = InputFlag::Video | InputFlag::Audio;

  char name[MAX_NAME];                                                        //	プラグインの名前
  char filepath[MAX_FILTER];                                                  //	入力ファイルフィルタ
  char information[256];                                                      //	プラグインの情報
  const char* extensions[10];                                                 //	拡張子リスト
  bool (*fn_init)();                                                          //	DLL開始時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  bool (*fn_exit)();                                                          //	DLL終了時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  INPUT_HANDLE (*fn_open)(const char* file);                                  //	入力ファイルをオープンする関数へのポインタ
  bool (*fn_close)(INPUT_HANDLE ih);                                          //	入力ファイルをクローズする関数へのポインタ
  bool (*fn_info_get)(INPUT_HANDLE ih, InputInfo* iip);                       //	入力ファイルの情報を取得する関数へポインタ
  int (*fn_set_frame)(INPUT_HANDLE ih, int frame);                            //	フレームを設定する関数へのポインタ
  int (*fn_get_frame)(INPUT_HANDLE ih);                                       //	現在のフレーム番号を取得する関数へのポインタ
  int (*fn_read_video)(INPUT_HANDLE ih, const InputInfo* iip, Movie* entity); //	画像データを読み込む関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	frame	: 読み込むフレーム番号
  //	buf		: データを読み込むバッファへのポインタ
  //	戻り値	: 読み込んだデータサイズ
  int (*fn_read_audio)(INPUT_HANDLE ih, int start, int length, void* buf); //	音声データを読み込む関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	start	: 読み込み開始サンプル番号
  //	length	: 読み込むサンプル数
  //	buf		: データを読み込むバッファへのポインタ
  //	戻り値	: 読み込んだサンプル数
  bool (*fn_is_keyframe)(INPUT_HANDLE ih, int frame); //	keyフレームか調べる関数
  bool (*fn_config_wnd)();                            // 設定ウィンドウを表示する関数へのポインタ(trueなら終了)
  int reserve[16];

  bool ext_supports(const char* ext) const {
    for(size_t i = 0; i < 10; ++i)
      if(extensions[i] && strncmp(extensions[i], ext, 10) == 0) return true;
  }
};

} // namespace mu
