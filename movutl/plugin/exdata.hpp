#pragma once
#include <movutl/asset/config.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/core/vector.hpp>
#include <string>

namespace mu {

using MULTI_THREAD_FUNC = void(*);
using HFONT = std::string;

//	ファイルインフォメーション構造体
struct FILE_INFO {
  int flag;                        //	ファイルのフグ
                                   //	FILE_INFO_FLAG_VIDEO	: 映像が存在する
                                   //	FILE_INFO_FLAG_AUDIO	: 音声が存在する
  std::string name;                //	ファイル名 ( avi_file_open()ではNULLになります )
  int w, h;                        //	元のサイズ
  int video_rate, video_scale;     //	フレームレート
  int audio_rate;                  //	音声サンプリングレート
  int audio_ch;                    //	音声チャンネル数
  int frame_n;                     //	総フレーム数
  std::string video_decode_format; //	ビデオ展開形式
  int video_decode_bit;            //	ビデオ展開形式のビット数
  int audio_n;                     //	音声の総サンプル数 ( avi_file_open()の時のみ設定されます )
  int reserve[4];                  //	拡張用に予約されてます
};

//	フレームステータス構造体
struct FRAME_STATUS {
  int video;      //	実際の映像データ番号
  int audio;      //	実際の音声データ番号
  int inter;      //	フレームのインターレース
                  //	FRAME_STATUS_INTER_NORMAL	: 標準
                  //	FRAME_STATUS_INTER_REVERSE	: 反転
                  //	FRAME_STATUS_INTER_ODD		: 奇数
                  //	FRAME_STATUS_INTER_EVEN		: 偶数
                  //	FRAME_STATUS_INTER_MIX		: 二重化
                  //	FRAME_STATUS_INTER_AUTO		: 自動
  int index24fps; //	現在は使用されていません
  int config;     //	フレームのプロファイル環境の番号
  int vcm;        //	フレームの圧縮設定の番号
  int edit_flag;  //	編集フラグ
                  //	EDIT_FRAME_EDIT_FLAG_KEYFRAME	: キーフレーム
                  //	EDIT_FRAME_EDIT_FLAG_MARKFRAME	: マークフレーム
                  //	EDIT_FRAME_EDIT_FLAG_DELFRAME	: 優先間引きフレーム
                  //	EDIT_FRAME_EDIT_FLAG_NULLFRAME	: コピーフレーム
  int reserve[9]; //	拡張用に予約されてます
};

//	外部関数構造体
struct ExeData {
  void (*get_ycp_ofs)(void* editp, int n, int ofs);
  //	※出来るだけget_ycp_source_cache()の方を使用するようにしてください
  //	指定したフレームのAVIファイル上でのオフセット分移動した
  //	フレーの画像データのポインタを取得します
  //	データはフィルタ前のものです
  //	editp 	: エディットハンドル
  //	n	 	: フレーム番号
  //	ofs	 	: フレームからのオフセット
  //  戻り値	: 画像データへのポインタ (NULLなら失敗)
  //			  画像データポインタの内容は次に外部関数を使うかメインに処理を戻すまで有効
  void* (*get_ycp)(void* editp, int n);
  //	※出来るだけget_ycp_source_cache()の方を使用するようにしてください
  //	指定したフレームの画像データのポインタを取得します
  //	データはフィルタ前のものです
  //	editp 	: エディットハンドル
  //	n	 	: フレーム番号
  //  戻り値	: 画像データへのポインタ (NULLなら失敗)
  //			  画像データポインタの内容は次に外部関数を使うかメインに処理を戻すまで有効
  void* (*get_pixelp)(void* editp, int n);
  //	指定したフレームのDIB形式(RGB24bit)の画像データのポインタを取得します
  //	データはフィルタ前のものです
  //	editp 	: エディットハンドル
  //	n		: フレーム番号
  //  戻り値	: DIB形式データへのポインタ (NULLなら失敗)
  //			  画像データポインタの内容は次に外部関数を使うかメインに処理を戻すまで有効
  int (*get_audio)(void* editp, int n, void* buf);
  //	指定したフレームのオーディオデータを読み込みます
  //	データはフィルタ前のものです
  //	editp 	: エディットハンドル
  //	n		: フレーム番号
  //	buf 	: 格納するバッファ (NULLならサンプル数の取得のみ)
  //  戻り値	: 読み込んだサンプル数
  bool (*is_editing)(void* editp);
  //	現在編集中か調べます
  //	editp 	: エディットハンドル
  //  戻り値	: TRUEなら編集中
  bool (*is_saving)(void* editp);
  //	現在保存中か調べます
  //	editp 	: エディットハンドル
  //  戻り値	: TRUEなら保存中
  int (*get_frame)(void* editp);
  //	現在の表示フレームを取得します
  //	editp 	: エディットハンドル
  //  戻り値	: 現在のフレーム番号
  int (*get_frame_n)(void* editp);
  //	総フレーム数を取得します
  //	editp 	: エディットハンドル
  //  戻り値	: 現在の総フレーム数
  bool (*get_frame_size)(void* editp, int* w, int* h);
  //	フィルタ前のフレームのサイズを取得します
  //	editp 	: エディットハンドル
  //	w,h 	: 画像サイズの格納ポインタ
  //  戻り値	: TRUEなら成功
  int (*set_frame)(void* editp, int n);
  //	現在の表示フレームを変更します
  //	editp 	: エディットハンドル
  //  n		: フレーム番号
  //  戻り値	: 設定されたフレーム番号
  int (*set_frame_n)(void* editp, int n);
  //	総フレーム数を変更します
  //	editp 	: エディットハンドル
  //  n		: フレーム数
  //  戻り値	: 設定された総フレーム数
  bool (*copy_frame)(void* editp, int d, int s);
  //	フレームを他のフレームにコピーします
  //	editp 	: エディットハンドル
  //	d	 	: コピー先フレーム番号
  //	s	 	: コピー元フレーム番号
  //  戻り値	: TRUEなら成功
  bool (*copy_video)(void* editp, int d, int s);
  //	フレームの映像だけを他のフレームにコピーします
  //	editp 	: エディットハンドル
  //	d	 	: コピー先フレーム番号
  //	s	 	: コピー元フレーム番号
  //  戻り値	: TRUEなら成功
  bool (*copy_audio)(void* editp, int d, int s);
  //	フレームの音声だを他のフレームにコピーします
  //	editp 	: エディットハンドル
  //	d	 	: コピー先フレーム番号
  //	s	 	: コピー元フレーム番号
  //  戻り値	: TRUEなら成功
  bool (*copy_clip)(void* hwnd, void* pixelp, int w, int h);
  //	クリップボードにDIB形式(RGB24bit)の画像をコピーします
  //	hwnd 	: ウィンドウハンドル
  //	pixelp	: DIB形式データへのポインタ
  //	w,h 	: 画像サイズ
  //  戻り値	: TRUEなら成功
  bool (*paste_clip)(void* hwnd, void* editp, int n);
  //	クリップボードから画像を張りつけます
  //	hwnd 	: ウィンドウハンドル
  //	editp 	: エディットハンドル
  //  n		: フレーム番号
  //  戻り値	: TRUEなら成功
  bool (*get_frame_status)(void* editp, int n, FRAME_STATUS* fsp);
  //	フレームのステータスを取得します
  //	editp 	: エディットハンドル
  //  n		: フレーム番号
  //  fps		: フレームステータスへのポインタ
  //  戻り値	: TRUEなら成功
  bool (*set_frame_status)(void* editp, int n, FRAME_STATUS* fsp);
  //	フレームのステータスを変更します
  //	editp 	: エディットハンドル
  //  n		: フレーム番号
  //  fps		: フレームステータスへのポインタ
  //  戻り値	: TRUEなら成功
  bool (*is_saveframe)(void* editp, int n);
  //	実際に保存されるフレームか調べます
  //	editp 	: エディットハンドル
  //  n		: フレーム番号
  //  戻り値	: TRUEなら保存されます
  bool (*is_keyframe)(void* editp, int n);
  //	キーフレームかどうか調べます
  //	editp 	: エディットハンドル
  //  n		: フレーム番号
  //  戻り値	: TRUEならキーフレーム
  bool (*is_recompress)(void* editp, int n);
  //	再圧縮が必要か調べます
  //	editp 	: エディットハンドル
  //  n		: フレーム番号
  //  戻り値	: TRUEなら再圧縮が必要
  bool (*filter_window_update)(void* fp);
  //	設定ウィンドウのトラックバーとチェックボックスを再描画します
  //	fp	 	: フィルタ構造体のポインタ
  //  戻り値	: TRUEなら成功
  bool (*is_filter_window_disp)(void* fp);
  //	設定ウィンドウが表示されているか調べます
  //	fp	 	: フィルタ構造体のポインタ
  //  戻り値	: TRUEなら表示されている
  bool (*get_file_info)(void* editp, FILE_INFO* fip);
  //	編集ファイルの情報を取得します
  //	editp 	: エディットハンドル
  //  fip		: ファイルインフォメーション構造体へのポインタ
  //  戻り値	: TRUEなら成功
  std::string (*get_config_name)(void* editp, int n);
  //	現在のプロファイルの名前を取得します
  //	editp 	: エディットハンドル
  //  n		: プロファイル環境の番号
  //  戻り値	: プロファイルの名前へのポインタ (NULLなら失敗)
  bool (*is_filter_active)(void* fp);
  //	フィルタが有効になっているか調べます
  //	fp	 	: フィルタ構造体のポインタ
  //  戻り値	: TRUEならフィルタ有効
  bool (*get_pixel_filtered)(void* editp, int n, void* pixelp, int* w, int* h);
  //	指定したフレームのDIB形式(RGB24bit)の画像データを読み込みます
  //	データはフィルタ後のものです
  //	editp 	: エディットハンドル
  //	n		: フレーム番号
  //  pixelp	: DIB形式データを格納するポインタ (NULLなら画像サイズだけを返します)
  //	w,h		: 画像のサイズ (NULLならDIB形式データだけを返します)
  //  戻り値	: TRUEなら成功
  int (*get_audio_filtered)(void* editp, int n, void* buf);
  //	指定したフレームのオーディオデータを読み込みます
  //	データはフィルタ後のものです
  //	editp* 	: エディットハンドル
  //	n		: フレーム番号
  //	buf 	: 格納するバッファ (NULLならサンプル数の取得のみ)
  //  戻り値	: 読み込んだサンプル数
  bool (*get_select_frame)(void* editp, int* s, int* e);
  //	選択開始終了フレームを取得します
  //	editp 	: エディットハンドル
  //	s		: 選択開始フレーム
  //	e		: 選択終了フレーム
  //  戻り値	: TRUEなら成功
  bool (*set_select_frame)(void* editp, int s, int e);
  //	選択開始終了フレームを設定します
  //	editp 	: エディットハンドル
  //	s		: 選択開始フレーム
  //	e		: 選択終了フレーム
  //  戻り値	: TRUEなら成功
  bool (*dlg_get_load_name)(std::string name, std::string filter, std::string def);
  //	ファイルダイアログを使って読み込むファイル名を取得します
  //	name	: ファイル名を格納するポインタ
  //	filter	: ファイルフィルタ
  //  def		: デフォルトのファイル名
  //  戻り値	: TRUEなら成功 FALSEならキャンセル
  bool (*dlg_get_save_name)(std::string name, std::string filter, std::string def);
  //	ファイルダイアログを使って書き込むファイル名を取得します
  //	name	: ファイル名を格納するポインタ
  //	filter	: ファイルフィルタ
  //  def		: デフォルトのファイル名
  //  戻り値	: TRUEなら成功 FALSEならキャンセル
  int (*ini_load_int)(void* fp, std::string key, int n);
  //	INIファイルから数値を読み込む
  //	fp	 	: フィルタ構造体のポインタ
  //	key		: アクセス用のキーの名前
  //  n		: デフォルトの数値
  //  戻り値	: 読み込んだ数値
  int (*ini_save_int)(void* fp, std::string key, int n);
  //	INIファイルに数値を書き込む
  //	fp	 	: フィルタ構造体のポインタ
  //	key		: アクセス用のキーの名前
  //  n		: 書き込む数値
  //  戻り値	: 書き込んだ数値
  bool (*ini_load_str)(void* fp, std::string key, std::string str, std::string def);
  //	INIファイルから文字列を読み込む
  //	fp	 	: フィルタ構造体のポインタ
  //	key		: クセス用のキーの名前
  //  str		: 文字列を読み込むバッファ
  //  def		: デフォルトの文字列
  //  戻り値	: TRUEなら成功
  bool (*ini_save_str)(void* fp, std::string key, std::string str);
  //	INIファイルに文字列を書き込む
  //	fp	 	: フィルタ構造体のポインタ
  //	key		: アクセス用のキーの名前
  //  n		: 書き込む文字列
  //  戻り値	: TRUEなら成功
  bool (*get_source_file_info)(void* editp, FILE_INFO* fip, int source_file_id);
  //	指定したファイルIDのファイルの情報を取得します
  //	editp 	: エディットハンドル
  //  fip		: ファイルインフォメーション構造体へのポインタ
  //	souce_file_id
  //			: ファイルID
  //  戻り値	: TRUEなら成功
  bool (*get_source_video_number)(void* editp, int n, int* source_file_id, int* source_video_number);
  //	指定したフレームのソースのファルIDとフレーム番号を取得します
  //	editp 	: エディットハンドル
  //	n		: フレーム番号
  //	souce_file_id
  //			: ファイルIDを格納するポインタ
  //	souce_video_number
  //			: フレーム番号を格納するポインタ
  //  戻り値	: TRUEなら成功
  bool (*get_config)(void* editp, Config* sip);
  //	システムの情報を取得します
  //	editp 	: エディットハンドル (NULLならsipの編集中のフラグとすべてのファイル名が無効になります)
  //  sip		: システムインフォメーション構造体へのポインタ
  //  戻り値	: TRUEなら成功
  void* (*get_filterp)(int filter_id);
  //	指定のフィルタIDのフィルタ構造体へのポインタを取得します
  //	filter_id
  //		 	: フィルタID (0～登録されてるフィルタの数-1までの値)
  //  戻り値	: フィルタ構造体へのポイタ (NULLなら失敗)
  void* (*get_ycp_filtering)(void* fp, void* editp, int n, void* reserve);
  //	※出来るだけget_ycp_filtering_cache_ex()の方を使用するようにしてください
  //	指定したフレームの画像データのポインタを取得します
  //	データは自分のフィルタの直前までフィルタしたものです
  //	fp	 	: フィルタ構造体のポインタ
  //	editp 	: エディットハンドル
  //	n	 	: フレーム番号
  //	reserve	: NULLを指定してください
  //  戻り値	: 画像データへのポインタ (NULLなら失敗)
  //			  画像データポインタの内容は次に外部関数を使うかメインに処理を戻すまで有効
  int (*get_audio_filtering)(void* fp, void* editp, int n, void* buf);
  //	指定したフレームのオーディオデータを読み込みます
  //	データは自分のフィルタの直前までフィルタしたものです
  //	fp	 	: フィルタ構造体のポインタ
  //	editp 	: エディットハンドル
  //	n		: フレーム番号
  //	buf 	: 格納するバッファ (NULLならサンプル数の取得のみ)
  //  戻り値	: 読み込んだサンプル数
  bool (*set_ycp_filtering_cache_size)(void* fp, int w, int h, int d, int flag);
  //	get_ycp_filtering_cache_ex()のキャッシュの設定をします
  //	設定値が変わった時のみキャッシュ領域を再確保します
  //	キャッシュ領域はフィルタがアクティブの時のみ確保されます
  //	fp	 	: フィルタ構造体のポインタ
  //	w	 	: キャッシュ領域の幅
  //	h	 	: キャッシュ領域の高さ
  //	d	 	: キャッシュするフレーム数
  //	flag 	: NULLを指定してください
  //  戻り値	: TRUEなら成功
  void* (*get_ycp_filtering_cache)(void* fp, void* editp, int n);
  //	※出来るだけget_ycp_filtering_cache_ex()の方を使用するようにしてください
  //	指定したフレームの画像データのキャッシュポインタを取得します
  //	set_ycp_filtering_cache_size()の設定にしたがってキャッシュされます
  //	データは自分のフィルタの直前までフィルタしたものです
  //	fp	 	: フィルタ構造体のポインタ
  //	editp 	: エディットハンドル
  //	n	 	: フレーム番号
  //  戻り値	: 画像データへのキャッシュポインタ (NULLなら失敗)
  //			  画像データポインタの内容はキャッシュから破棄されるまで有効
  void* (*get_ycp_source_cache)(void* editp, int n, int ofs);
  //	指定したフレームの画像データのポインタを取得します
  //	データはフィルタ前のものです
  //	editp 	: エディットハンドル
  //	n	 	: フレーム番号
  //	ofs	 	: 元のAVI上でのフレームのオフセット
  //  戻り値	: 画像データへのポインタ (NULLなら失敗)
  //			  画像データポインタの内容はキャッシュから破棄されるまで有効
  void* (*get_disp_pixelp)(void* editp, std::string format);
  //	表示されているフレームの画像データのポインタを取得します
  //	データはフィルタ後のものです
  //	表示フィルタのみ使用可能です。
  //	editp 	: エディットハンドル
  //	format	: 画像フォーマット( NULL = RGB24bit / 'Y''U''Y''2' = YUY2 )
  //  戻り値	: 画像データへのポインタ (NULLなら失敗)
  //			  画像データポインタの内容は次に外部関数を使うかメインに処理を戻すまで有効
  bool (*get_pixel_source)(void* editp, int n, void* pixelp, std::string format);
  //	指定したフレームの画像データを読み込みます
  //	データはフィルタ前のものです
  //	editp 	: エィットハンドル
  //	n	 	: フレーム番号
  //  pixelp	: DIB形式データを格納するポインタ
  //	format	: 画像フォーマット( NULL = RGB24bit / 'Y''U''Y''2' = YUY2 )
  //  戻り値	: TRUEなら成功
  bool (*get_pixel_filtered_ex)(void* editp, int n, void* pixelp, int* w, int* h, std::string format);
  //	指定したフレームの画像データを読み込みます
  //	データはフィルタ後のものです
  //	editp 	: エディットハンドル
  //	n	 	: フレーム番号
  //  pixelp	: DIB形式データを格納するポインタ (NULLなら画像サイズだけを返します)
  //	w,h		: 画像のサイズ (NULLならDIB形式データだけを返します)
  //	format	: 画像フォーマット( NULL = RGB24bit / 'Y''U''Y''2' = YUY2 )
  //  戻り値	: TRUEなら成功
  Image* (*get_ycp_filtering_cache_ex)(void* fp, void* editp, int n, int* w, int* h);
  //	指定したフレームの画データのキャッシュポインタを取得します
  //	set_ycp_filtering_cache_size()の設定にしたがってキャッシュされます
  //	データは自分のフィルタの直前までフィルタしたものです
  //	fp	 	: フィルタ構造体のポインタ
  //	editp 	: エディットハンドル
  //	n	 	: フレーム番号
  //	w,h		: 取得した画像のサイズ (NULLなら無視されます)
  //  戻り値	: 画像データへのキャッシュポインタ (NULLなら失敗)
  //			  画像データポインタの内容はキャッシュから破棄されるまで有効
  bool (*exec_multi_thread_func)(MULTI_THREAD_FUNC func, void* param1, void* param2);
  //	指定した関数をシステムの設定値に応じたスレッド数で呼び出します
  //	呼び出された関数内からWin32APIや外部関数(rgb2yc,yc2rgbは除く)を使用しないでください
  //	func	: マルチスレッドで呼出す関数
  //	param1 	: 呼び出す関数に渡す汎用パラメータ
  //	param2 	: 呼び出す関数に渡す汎用パラメータ
  //  戻り値	: TRUEなら成功
  Image* (*create_yc)(void);
  //	空のフレーム画像データ領域を作成します
  //	ycp_editと同様に外部関数で使用できますが
  //	FILTER_PROC_INFOのycp_edit,ycp_tempと入れ替えることは出来ません
  //  戻り値	: 作成したフレーム画像データへのポインタ (NULLなら失敗)
  void (*delete_yc)(Image* ycp);
  //	create_ycで作成した領域を削除します
  bool (*load_image)(Image* ycp, std::string file, int* w, int* h, int flag);
  //	フレーム画像データにBMPファイルから画像を読み込みます
  //	ycp     : 画像を読み込むフレーム画像へのポインタ (NULLなら描画をせずにサイズを返します)
  //	file	: 読み込むBMPファイル名
  //	w,h		: 読み込んだ画像のサイズ (NULLを指定できます)
  //	flag 	: NULLを指定してください
  //  戻り値	: TRUEなら成功
  void (*resize_yc)(Image* ycp, int w, int h, Image* ycp_src, int sx, int sy, int sw, int sh);
  //	フレーム画像データをリサイズします
  //	元画像の任意の画像領域をリサイズすることも出来ます
  //	ycp     : リサイズ後のフレーム画像を格納するポインタ
  //	w,h     : リサイズの解像度
  //	ycp_src	: 元画像のフレーム画像へのポインタ(NULLならycpと同じ)
  //	sx,sy	: 元画像のリサイズ対象領域の左上の座標
  //	sw,sh	: 元画像のリサイズ対象領域のサイズ
  void (*copy_yc)(Image* ycp, int x, int y, Image* ycp_src, int sx, int sy, int sw, int sh, int tr);
  //	フレーム画像データの任意の領域をコピーします
  //	描画の際は最大画像サイズの領域に収まるようにリッピングをします
  //	コピー元とコピー先の領域は重ならないようにしてください
  //	ycp     : コピー先のフレーム画像へのポインタ
  //	x,y		: コピー先の左上の座標
  //	ycp_src	: コピー元のフレーム画像へのポインタ
  //	sx,sy	: コピー元の左上の座標
  //	sw,sh	: コピー元のサイズ
  //	tr      : コピー元の透明度 (0～4096)
  void (*draw_text)(Image* ycp, int x, int y, std::string text, int r, int g, int b, int tr, HFONT hfont, int* w, int* h);
  //	フレーム画像データにテキストを描画します
  //	描画の際は最大画像サイズの領域に収まるようにクリッピングをします
  //	ycp     : 描画するフレーム画像データへのポインタ (NULLなら描画をせずにサイズを返します)
  //	x,y		: 描画する左上の座標
  //	text	: 描画するテキストの内容
  //	r,g,b	: 描画色 (0～255)
  //	tr      : 透明度 (0～4096)
  //	hfont	: 描画で使用するフォント (NULLならデフォルトのフォント)
  //	w,h		: 描画したテキスト領域のサイズ (NULLを指定できます)
  std::string (*get_avi_file_filter)(int type);
  //	avi_file_open()で読み込めるファイルのファイルィルタを取得します
  //	type	: ファイルの種類
  //	GET_AVI_FILE_FILTER_TYPE_VIDEO	: ビデオル
  //	GET_AVI_FILE_FILTER_TYPE_AUDIO	: オーディオ
  //  戻り値	: ファイルフィルタへのポインタ
  uint8_t* (*get_frame_status_table)(void* editp, int type);
  //	フレームのステータスが格納されているバッファへのポインタを取得します
  //	editp 	: エディットハンドル
  //  type	: ステータスの種類
  //	FARME_STATUS_TYPE_EDIT_FLAG	: 編集フラグ
  //	FARME_STATUS_TYPE_INTER		: インターレース
  //  戻り値	: バッファへのポインタ
  //			  バッファへのポインタの内容は編集ファイルがクローズされるまで有効
  bool (*set_undo)(void* editp);
  //	現在の編集状況をアンドゥバッファに設定します
  //	editp 	: エディットハンドル
  //  戻り値	: TRUEなら成功
  bool (*add_menu_item)(void* fp, std::string name, void* hwnd, int id, int def_key, int flag);
  //	メインウィンドウの設定メニュー項目を追加します
  //	メニューが選択された時にhwndで指定したウィンドウに
  //	WM_FILTER_COMMANDのメッセージを送ります
  //	※必ずfunc_init()かWM_FILTER_INITから呼び出すようにしてください。
  //	fp	 	: フィルタ構造体のポインタ
  //	name 	: メニューの名前
  //	hwnd 	: WM_FILTER_COMMANDを送るウィンドウハンドル
  //	id	 	: WM_FILTER_COMMANDのWPARAM
  //	def_key	: 標準のショートカットキーの仮想キーコード (NULLなら無し)
  //	flag	: フラグ
  //	ADD_MENU_ITEM_FLAG_KEY_SHIFT	: 標準のショートカットキーをSHIFT+キーにする
  //	ADD_MENU_ITEM_FLAG_KEY_CTRL		: 標準のショートカットキーをCTRL+キーにする
  //	ADD_MENU_ITEM_FLAG_KEY_ALT		: 標準のショートカットキーをALT+キーにする
  //  戻り値	: TRUEなら成功
  bool (*edit_open)(void* editp, std::string file, int flag);
  //	編集ァイルを開きます
  //	editp 	: エディットハンドル
  //	file 	: ファイル名
  //	flag 	: フラグ
  //	EDIT_OPEN_FLAG_ADD			: 追加読み込みをします
  //	EDIT_OPEN_FLAG_AUDIO		: 音声読み込みをします
  //	EDIT_OPEN_FLAG_PROJECT		: プロジェクトファイルを開きます
  //	EDIT_OPEN_FLAG_DIALOG		: 読み込みダイアログを表示します
  //  戻り値	: TRUEなら成功
  bool (*edit_close)(void* editp);
  //	編集ファイルを閉じます
  //	editp 	: エディットハンドル
  //  戻り値	: TRUEなら成功
  bool (*edit_output)(void* editp, std::string file, int flag, std::string type);
  //	編集データをAVI出力します
  //	WAV出力やプラグイン出力も出来ます
  //	editp 	: エディットハンドル
  //	file 	: 出力ファイル名
  //	flag	: フラグ
  //	EDIT_OUTPUT_FLAG_NO_DIALOG	: 出力ダイアログを表示しませ
  //	EDIT_OUTPUT_FLAG_WAV		: WAV出力をします
  //	type	: 出力プラグインの名前 (NULLならAVI/WAV出力)
  //  戻り値	: TRUEなら成功
  bool (*set_config)(void* editp, int n, std::string name);
  //	プロファイルを設定します
  //	editp 	: エディットハンドル
  //  n		: プロファイル環境の番号
  //  name	: プロファイルの名前
  //  戻り値	: TRUEなら成功
  int reserve[7];
};
} // namespace mu
