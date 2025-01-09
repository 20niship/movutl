#pragma once

#include <movutl/asset/image.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/props.hpp>
#include <movutl/plugin/exdata.hpp>

namespace mu {

struct FilterInData {
public:
  int flag;          //	フィルタのフラグ
                     //	FILTER_PROC_INFO_FLAG_INVERT_FIELD_ORDER	: フィールドオーダーを標準と逆に扱う ( 標準はボトム->トップになっています )
                     //	FILTER_PROC_INFO_FLAG_INVERT_INTERLACE		: 解除方法を反転する ( インターレース解除フィルタのみ )
  Image* ycp_edit;   //	画像データへのポインタ ( ycp_editとycp_tempは入れ替えれます )
  Image* ycp_temp;   //	テンポラリ領域へのポインタ
  int w, h;          //	現在の画像のサイズ ( 画像サイズは変更出来ます )
  int max_w, max_h;  //	画像領域のサイズ
  int frame;         //	現在のフレーム番号( 番号は0から )
  int frame_n;       //	総フレーム数
  int org_w, org_h;  //	元の画像のサイズ
  short* audiop;     //	オーディオデータへのポインタ ( オーディオフィルタの時のみ )
                     //	オーディオ形式はPCM16bitです ( 1サンプルは mono = 2byte , stereo = 4byte )
  int audio_n;       //	オーディオサンプルの総数
  int audio_ch;      //	オーディオチャンネル数
  Ref<Image> pixelp; //	現在は使用されていません
  ExeData* editp;    //	エディットハンドル
  int yc_size;       //	画像領域の画素のバイトサイズ
  int line_size;     //	画像領域の幅のバイトサイズ
  int reserve[8];    //	拡張用に予約されてます
};

enum FilterInfoType {
  FilterAlwaysActive = 0,
  Filter_ManualRedraw,
  Filter_NoInitData,
  // TODO: AEとかの設定を読み込んでくる
};

struct FilterPluginTable {
  uint64_t guid;
  FilterInfoType flag;
  std::string name;
  std::string infomation;
  uint32_t version = 0;
  std::string version_str;

  PropInfos (*fn_get_props)();

  void (*fn_cutstom_wnd)() = nullptr;
  void (*fn_update_value)();
  bool (*fn_init)(void* fp, void* editp); //	開始時に呼ばれる関数へのポインタ (NULLなら呼ばれせん)
  bool (*fn_exit)(void* fp);              //	終了時に呼ばれる関数へのポインタ (NULLなら呼ばれません)

  bool (*fn_proc)(void* fp, FilterInData* fpip, const Props& p); //	フィルタ処理関数へのポインタ (NULLなら呼ばれません)
  bool (*fn_update)(void* fp, int status);
  //	自分の設定が変更されたときに呼ばれる関数へのポインタ (NULLなら呼ばれません)
  //	FILTER_UPDATE_STATUS_ALL		: 全項目が変更された
  //	FILTER_UPDATE_STATUS_TRACK + n	: n番目のトラックバーが変更された
  //	FILTER_UPDATE_STATUS_CHECK + n	: n番目のチェックボックスが変更された

  bool (*func_is_saveframe)(void* fp, void* editp, int saveno, int frame, int fps, int edit_flag, int inter);
  //	インターレース解除フィルタで保存するフレームを決める時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  //	saveno		: セーブする範囲の先頭からのフレーム番号
  //	frame		: 編集フレーム番号
  //	fps			: フレームレートの変更の設定値 (30,24,20,15,10)
  //	edit_flag	: 編集フラグ
  //	inter		: フレームのインターレース
  //	戻り値		: TRUEを返すと保存フレーム、FALSEを返すと間引きフレームになります。

  bool (*fn_project_load)(void* fp, void* editp, void* data, int size);
  //	プロジェクトファイルからデータを読み込んだ時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  //	プロジェクトファイルに保存したデータが無い場合は呼ばれません
  //	data 	: プロジェクトから読み込んだデータへのポインタ
  //	size 	: プロジェクトから読み込んだデータのバイト
  //  戻り値	: 成功ならTRUE

  bool (*func_project_save)(void* fp, void* editp, void* data, int* size);

  //	メインウィンドウのタイトルバーを表示する時に呼ばれる関へのポインタ (NULLなら呼ばれません)
  //	タイトルバーの文字列を変更できます (未編集時、出力時は呼ばれません)
  //	frame		: 編集フレーム番号
  //	title 		: 表示するタイトルバーの文字列
  //	max_title 	: titleのバッファサイズ
  //  戻り値	: 成功ならTRUE
  int reserve[2]; //	拡張用に予約されてます。NULLにしてください。
};
} // namespace mu
