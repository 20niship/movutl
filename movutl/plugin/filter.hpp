#pragma once

#include <movutl/app/exdata.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/props.hpp>

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

struct FilterInfo {
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

#if 0

  /************************************************************************
  ************************************************************************
  ************************************************************************
  ************************************************************************
  ************************************************************************
  ***********************************************************************/
#define FRAME_STATUS_INTER_NORMAL 0
#define FRAME_STATUS_INTER_REVERSE 1
#define FRAME_STATUS_INTER_ODD 2
#define FRAME_STATUS_INTER_EVEN 3
#define FRAME_STATUS_INTER_MIX 4
#define FRAME_STATUS_INTER_AUTO 5
#define EDIT_FRAME_EDIT_FLAG_KEYFRAME 1
#define EDIT_FRAME_EDIT_FLAG_MARKFRAME 2
#define EDIT_FRAME_EDIT_FLAG_DELFRAME 4
#define EDIT_FRAME_EDIT_FLAG_NULLFRAME 8
#define FILE_INFO_FLAG_VIDEO 1
#define FILE_INFO_FLAG_AUDIO 2

#define SYS_INFO_FLAG_EDIT 1
#define SYS_INFO_FLAG_VFAPI 2
#define SYS_INFO_FLAG_USE_SSE 4
#define SYS_INFO_FLAG_USE_SSE2 8

//	マルチスレッド関数用の定義
typedef void (*MULTI_THREAD_FUNC)(int thread_id, int thread_num, void* param1, void* param2);
//	thread_id	: スレッド番号 ( 0 ～ thread_num-1 )
//	thread_num	: スレッド数 ( 1 ～ )
//	param1		: 汎用パラメータ
//	param2		: 汎用パラメータ

//	AVI入力ファイルハンドル
typedef void* AVI_FILE_HANDLE;

#define AVI_FILE_OPEN_FLAG_VIDEO_ONLY 16
#define AVI_FILE_OPEN_FLAG_AUDIO_ONLY 32
#define AVI_FILE_OPEN_FLAG_ONLY_YUY2 0x10000
#define AVI_FILE_OPEN_FLAG_ONLY_RGB24 0x20000
#define AVI_FILE_OPEN_FLAG_ONLY_RGB32 0x40000
#define GET_AVI_FILE_FILTER_TYPE_VIDEO 0
#define GET_AVI_FILE_FILTER_TYPE_AUDIO 1
#define FARME_STATUS_TYPE_EDIT_FLAG 0
#define FARME_STATUS_TYPE_INTER 1
#define ADD_MENU_ITEM_FLAG_KEY_SHIFT 1
#define ADD_MENU_ITEM_FLAG_KEY_CTRL 2
#define ADD_MENU_ITEM_FLAG_KEY_ALT 4
#define EDIT_OPEN_FLAG_ADD 2
#define EDIT_OPEN_FLAG_AUDIO 16
#define EDIT_OPEN_FLAG_PROJECT 512
#define EDIT_OPEN_FLAG_DIALOG 65536
#define EDIT_OUTPUT_FLAG_NO_DIALOG 2
#define EDIT_OUTPUT_FLAG_WAV 4
#define FILTER_FLAG_ACTIVE 1
#define FILTER_FLAG_ALWAYS_ACTIVE 4
#define FILTER_FLAG_CONFIG_POPUP 8
#define FILTER_FLAG_CONFIG_CHECK 16
#define FILTER_FLAG_CONFIG_RADIO 32
#define FILTER_FLAG_EX_DATA 1024
#define FILTER_FLAG_PRIORITY_HIGHEST 2048
#define FILTER_FLAG_PRIORITY_LOWEST 4096
#define FILTER_FLAG_WINDOW_THICKFRAME 8192
#define FILTER_FLAG_WINDOW_SIZE 16384
#define FILTER_FLAG_DISP_FILTER 32768
#define FILTER_FLAG_REDRAW 0x20000
#define FILTER_FLAG_EX_INFORMATION 0x40000
#define FILTER_FLAG_INFORMATION 0x80000
#define FILTER_FLAG_NO_CONFIG 0x100000
#define FILTER_FLAG_AUDIO_FILTER 0x200000
#define FILTER_FLAG_RADIO_BUTTON 0x400000
#define FILTER_FLAG_WINDOW_HSCROLL 0x800000
#define FILTER_FLAG_WINDOW_VSCROLL 0x1000000
#define FILTER_FLAG_INTERLACE_FILTER 0x4000000
#define FILTER_FLAG_NO_INIT_DATA 0x8000000
#define FILTER_FLAG_IMPORT 0x10000000
#define FILTER_FLAG_EXPORT 0x20000000
#define FILTER_FLAG_MAIN_MESSAGE 0x40000000
#define WM_FILTER_UPDATE (WM_USER + 100)
#define WM_FILTER_FILE_OPEN (WM_USER + 101)
#define WM_FILTER_FILE_CLOSE (WM_USER + 102)
#define WM_FILTER_INIT (WM_USER + 103)
#define WM_FILTER_EXIT (WM_USER + 104)
#define WM_FILTER_SAVE_START (WM_USER + 105)
#define WM_FILTER_SAVE_END (WM_USER + 106)
#define WM_FILTER_IMPORT (WM_USER + 107)
#define WM_FILTER_EXPORT (WM_USER + 108)
#define WM_FILTER_CHANGE_ACTIVE (WM_USER + 109)
#define WM_FILTER_CHANGE_WINDOW (WM_USER + 110)
#define WM_FILTER_CHANGE_PARAM (WM_USER + 111)
#define WM_FILTER_CHANGE_EDIT (WM_USER + 112)
#define WM_FILTER_COMMAND (WM_USER + 113)
#define WM_FILTER_FILE_UPDATE (WM_USER + 114)
#define WM_FILTER_MAIN_MOUSE_DOWN (WM_USER + 120)
#define WM_FILTER_MAIN_MOUSE_UP (WM_USER + 121)
#define WM_FILTER_MAIN_MOUSE_MOVE (WM_USER + 122)
#define WM_FILTER_MAIN_KEY_DOWN (WM_USER + 123)
#define WM_FILTER_MAIN_KEY_UP (WM_USER + 124)
#define WM_FILTER_MAIN_MOVESIZE (WM_USER + 125)
#define WM_FILTER_MAIN_MOUSE_DBLCLK (WM_USER + 126)
#define WM_FILTER_MAIN_MOUSE_R_DOWN (WM_USER + 127)
#define WM_FILTER_MAIN_MOUSE_R_UP (WM_USER + 128)
#define WM_FILTER_MAIN_MOUSE_WHEEL (WM_USER + 129)
#define WM_FILTER_MAIN_CONTEXTMENU (WM_USER + 130)
#define FILTER_UPDATE_STATUS_ALL 0
#define FILTER_UPDATE_STATUS_TRACK 0x10000
#define FILTER_UPDATE_STATUS_CHECK 0x20000
#define FILTER_WINDOW_SIZE_CLIENT 0x10000000
#define FILTER_WINDOW_SIZE_ADD 0x30000000

//	フィルタDLL用構造体
typedef struct {
  int flag;
  int x, y;
  TCHAR* name;
  int track_n;
  TCHAR** track_name;
  int* track_default;
  int *track_s, *track_e;
  int check_n;
  TCHAR** check_name;
  int* check_default;
  bool (*func_proc)(FILTER* fp, FILTER_PROC_INFO* fpip);
  bool (*func_init)(FILTER* fp);
  bool (*func_exit)(FILTER* fp);
  bool (*func_update)(FILTER* fp, int status);
  bool (*func_WndProc)(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, void* editp, FILTER* fp);
  int *track, *check;
  void* ex_data_ptr;
  int ex_data_size;
  TCHAR* information;
  bool (*func_save_start)(FILTER* fp, int s, int e, void* editp);
  bool (*func_save_end)(FILTER* fp, void* editp);
  EXFUNC* exfunc;
  HWND hwnd;
  HINSTANCE dll_hinst;
  void* ex_data_def;
  bool (*func_is_saveframe)(FILTER* fp, void* editp, int saveno, int frame, int fps, int edit_flag, int inter);
  bool (*func_project_load)(FILTER* fp, void* editp, void* data, int size);
  bool (*func_project_save)(FILTER* fp, void* editp, void* data, int* size);
  bool (*func_modify_title)(FILTER* fp, void* editp, int frame, LPSTR title, int max_title);
  TCHAR* dll_path;
  int reserve[2];
} FILTER_DLL;

#define MID_FILTER_BUTTON 12004

bool func_proc(FILTER* fp, FILTER_PROC_INFO* fpip);
bool func_init(FILTER* fp);
bool func_exit(FILTER* fp);
bool func_update(FILTER* fp, int status);
bool func_WndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, void* editp, FILTER* fp);
bool func_save_start(FILTER* fp, int s, int e, void* editp);
bool func_save_end(FILTER* fp, void* editp);
bool func_is_saveframe(FILTER* fp, void* editp, int saveno, int frame, int fps, int edit_flag, int inter);
bool func_project_load(FILTER* fp, void* editp, void* data, int size);
bool func_project_save(FILTER* fp, void* editp, void* data, int* size);
bool func_modify_title(FILTER* fp, void* editp, int frame, LPSTR title, int max_title);

#endif
