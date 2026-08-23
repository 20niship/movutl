#pragma once
#include <cstdint>
#include <cstring>
#include <movutl/asset/entity.hpp>
#include <movutl/core/defines.hpp>

namespace mu {

#define MAX_NAME 64
#define MAX_SUPPORTED_EXT 10

/// 入力プラグインのフラグ (aviutl2 SDK の INPUT_PLUGIN_TABLE 方針に準拠)
enum InputPluginFlag {
  InputPluginFlag_Video      = 1,  ///< 映像をサポートする
  InputPluginFlag_Audio      = 2,  ///< 音声をサポートする
  InputPluginFlag_Concurrent = 16, ///< 同一ハンドルの各関数が同時に呼ばれることをサポートする
};

//	入力プラグイン構造体
//
//	aviutl2 SDK (ext/aviutl2_sdk_mirror/include/aviutl2_sdk/input2.h) の方針に準拠した設計:
//	- フレーム指定読み込み: fn_read_video(handle, frame, buf) が呼ばれた際に、
//	  プラグイン内部でシーク+デコードを行い、呼び出し側が確保したバッファへ書き込む
//	- ハンドルはファイル毎に独立しており、複数の動画を同時に読み込める
//	- fn_set_frame / fn_get_frame 等のステートフルな関数は廃止
struct InputPluginTable {
  uint64_t guid;
  int flag = InputPluginFlag_Video;                                           //	InputPluginFlag の組み合わせ
  EntityType supports = EntityType_Movie | EntityType_Audio;                  //	サポートしているオブジェクトの種類
  char name[MAX_NAME];                                                        //	プラグインの名前
  char filepath[MAX_FILTER];                                                  //	入力ファイルフィルタ
  char information[256];                                                      //	プラグインの情報
  const char* extensions[MAX_SUPPORTED_EXT];                                  //	拡張子リスト
  bool (*fn_init)();                                                          //	DLL開始時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  bool (*fn_exit)();                                                          //	DLL終了時に呼ばれる関数へのポインタ (NULLなら呼ばれません)

  //	入力ファイルをオープンする関数へのポインタ
  //	file	: ファイル名
  //	戻り値	: 入力ファイルハンドル ※失敗時はnullptrを返却
  InputHandle (*fn_open)(const char* file);

  //	入力ファイルをクローズする関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	戻り値	: 成功時はtrueを返却
  bool (*fn_close)(InputHandle ih);

  //	入力ファイルの情報を取得する関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	iip		: 入力ファイル情報構造体へのポインタ
  //	戻り値	: 成功時はtrueを返却
  bool (*fn_info_get)(InputHandle ih, EntityInfo* iip);

  //	画像データを読み込む関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	frame	: 読み込むフレーム番号
  //	buf		: データを読み込むバッファへのポインタ (info.width * info.height * 4 byte 以上が必要)
  //	戻り値	: 読み込んだデータサイズ (失敗時は 0 以下)
  int (*fn_read_video)(InputHandle ih, int frame, void* buf);

  //	音声データを読み込む関数へのポインタ
  //	ih		: 入力ファイルハンドル
  //	start	: 読み込み開始サンプル番号
  //	length	: 読み込むサンプル数
  //	buf		: データを読み込むバッファへのポインタ
  //	戻り値	: 読み込んだサンプル数
  int (*fn_read_audio)(InputHandle ih, int start, int length, void* buf);

  bool (*fn_is_keyframe)(InputHandle ih, int frame); //	keyフレームか調べる関数 (NULLなら全てキーフレーム)
  bool (*fn_config_wnd)();                           //	設定ウィンドウを表示する関数へのポインタ(trueなら終了)
  int reserve[16];

  bool ext_supports(const char* ext) const {
    if(ext == nullptr) return false;
    for(size_t i = 0; i < MAX_SUPPORTED_EXT; ++i)
      if(extensions[i] && strncmp(extensions[i], ext, 10) == 0) return true;
    return false;
  }
  bool is_supports(EntityType type) const { return (supports & type) != 0; }
};

} // namespace mu
