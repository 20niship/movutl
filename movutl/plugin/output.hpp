#pragma once
#include <cstring>
#include <cutil/prop.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/plugin/input.hpp> // MAX_NAME, MAX_SUPPORTED_EXT

namespace mu {

// 出力(エクスポート)プラグイン構造体。InputPluginTable/FilterPluginTableと同じ素朴なCテーブル設計。
struct OutputPluginTable {
  uint64_t guid;
  char name[MAX_NAME];                       //	プラグインの名前
  char information[256];                     //	プラグインの情報
  bool is_sequence = false;                  //	true: フレーム毎に別ファイルへ書き出す(png連番等) / false: 1ファイルへ連続して書き込む(mp4等)
  const char* extensions[MAX_SUPPORTED_EXT]; //	対応拡張子

  //	エクスポート開始前に呼ばれ、設定可能なパラメータをpropsに登録し、defaultsへ初期値を設定する(NULLなら呼ばれません)
  bool (*fn_init)(cutil::PropInfo* props, cutil::Prop* defaults) = nullptr;
  bool (*fn_exit)()                                              = nullptr; //	DLL終了時に呼ばれる関数へのポインタ(NULLなら呼ばれません)

  // 出力を開始する関数へのポインタ(pathはis_sequence時はフレーム番号埋め込み前のベースパス。audio_sample_rate=0は音声なしの意)。戻り値は出力ハンドル(失敗時nullptr)
  void* (*fn_open)(const char* path, int width, int height, float framerate, int audio_sample_rate, int audio_channels, const cutil::Prop& props) = nullptr;

  // 1フレーム書き込む関数へのポインタ。imgはBGRA8(Image::data()と同じ並び)、frameは0始まりのエクスポート範囲内相対番号
  bool (*fn_write_frame)(void* handle, const Image* img, int frame) = nullptr;

  // 音声データを書き込む関数へのポインタ(音声非対応、またはfn_openにaudio_sample_rate=0を渡した場合はNULLのまま)。PCM16 interleaved
  bool (*fn_write_audio)(void* handle, const int16_t* pcm, int n_samples) = nullptr;

  //	出力を終了する関数へのポインタ(mp4等はここでmuxのtrailerを書く)
  bool (*fn_close)(void* handle) = nullptr;

  int reserve[8];

  cutil::PropInfo props; // フィールドのウィジェット表示情報(fn_initが埋める)
  cutil::Prop defaults;  // fn_initが設定するフィールドの初期値

  bool ext_supports(const char* ext) const {
    if(ext == nullptr) return false;
    for(size_t i = 0; i < MAX_SUPPORTED_EXT; ++i)
      if(extensions[i] && strncmp(extensions[i], ext, 10) == 0) return true;
    return false;
  }
};

} // namespace mu
