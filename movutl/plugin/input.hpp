#pragma once
#include <cstdint>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/array.hpp>
#include <movutl/core/defines.hpp>


namespace mu {


#define MAX_NAME 64
#define MAX_SUPPORTED_EXT 10

//	入力プラグイン構造体
struct InputPluginTable {
  uint64_t guid;
  EntityType supports = EntityType_Movie | EntityType_Audio;                  //	サポートしているオブジェクトの種類
  char name[MAX_NAME];                                                        //	プラグインの名前
  char filepath[MAX_FILTER];                                                  //	入力ファイルフィルタ
  char information[256];                                                      //	プラグインの情報
  const char* extensions[MAX_SUPPORTED_EXT];                                  //	拡張子リスト
  bool (*fn_init)();                                                          //	DLL開始時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  bool (*fn_exit)();                                                          //	DLL終了時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  InputHandle (*fn_open)(const char* file);                                   //	入力ファイルをオープンする関数へのポインタ
  bool (*fn_close)(InputHandle ih);                                           //	入力ファイルをクローズする関数へのポインタ
  bool (*fn_info_get)(InputHandle ih, EntityInfo* iip);                       //	入力ファイルの情報を取得する関数へポインタ
  int (*fn_set_frame)(InputHandle ih, int frame);                             //	フレームを設定する関数へのポインタ
  int (*fn_get_frame)(InputHandle ih);                                        //	現在のフレーム番号を取得する関数へのポインタ
  int (*fn_read_video)(InputHandle ih, const EntityInfo* iip, Movie* entity); //	画像データを読み込む関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	frame	: 読み込むフレーム番号
  //	buf		: データを読み込むバッファへのポインタ
  //	戻り値	: 読み込んだデータサイズ
  int (*fn_read_audio)(InputHandle ih, int start, int length, void* buf); //	音声データを読み込む関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	start	: 読み込み開始サンプル番号
  //	length	: 読み込むサンプル数
  //	buf		: データを読み込むバッファへのポインタ
  //	戻り値	: 読み込んだサンプル数
  bool (*fn_is_keyframe)(InputHandle ih, int frame); //	keyフレームか調べる関数
  bool (*fn_config_wnd)();                           // 設定ウィンドウを表示する関数へのポインタ(trueなら終了)
  int reserve[16];

  bool ext_supports(const char* ext) const {
    for(size_t i = 0; i < MAX_SUPPORTED_EXT; ++i)
      if(extensions[i] && strncmp(extensions[i], ext, 10) == 0) return true;
  }
  bool is_supports(EntityType type) const { return (supports & type) != 0; }
};

} // namespace mu
