#pragma once

#include <movutl/asset/image.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/props.hpp>
#include <movutl/plugin/exdata.hpp>

namespace mu {

struct FilterInData {
public:
  int flag;                     //	フィルタのフラグ
                                //	FILTER_PROC_INFO_FLAG_INVERT_FIELD_ORDER	: フィールドオーダーを標準と逆に扱う ( 標準はボトム->トップになっています )
                                //	FILTER_PROC_INFO_FLAG_INVERT_INTERLACE		: 解除方法を反転する ( インターレース解除フィルタのみ )
  ImageRGBA* img = nullptr;     //	画像データへのポインタ ( ycp_editとycp_tempは入れ替えれます )
  Composition* compo = nullptr; //	コンポジション
  Entity* entt = nullptr;       //	フィルタが適用されるエンティティ
  Vec2d max_size;               //	画像領域のサイズ
  Vec2d org_size;               //	オリジナル画像のサイズ
  short* audiop = nullptr;      //	オーディオデータへのポインタ ( オーディオフィルタの時のみ )
                                //	オーディオ形式はPCM16bitです ( 1サンプルは mono = 2byte , stereo = 4byte )
  int audio_n = 0;              //	オーディオサンプルの総数
  int audio_ch = 0;             //	オーディオチャンネル数
  int reserve[8];               //	拡張用に予約されてます
};

enum FilterInfoType {
  FilterDefault = 0,
  FilterAlwaysActive,
  Filter_ManualRedraw,
  Filter_NoInitData,
  // TODO: AEとかの設定を読み込んでくる
};

struct FilterPluginTable {
  uint64_t guid;
  FilterInfoType flag;
  FixString name;
  FixStringBase<256> info;
  uint32_t version = 0;
  std::string version_str;

  void (*fn_cutstom_wnd)() = nullptr;
  void (*fn_update_value)();
  bool (*fn_init)(void* fp, ExeData* editp, PropsInfo* props); //	開始時に呼ばれる関数へのポインタ (NULLなら呼ばれせん)
  bool (*fn_exit)(void* fp);                                   //	終了時に呼ばれる関数へのポインタ (NULLなら呼ばれません)

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

  PropsInfo props;
};
} // namespace mu
