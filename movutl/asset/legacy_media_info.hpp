#pragma once
#include <string>

namespace mu {

// AviUtl互換目的で保持している未使用構造体(現状どこからも参照されていない)。

//	ファイルインフォメーション構造体
struct FILE_INFO {
  int flag;                        //	ファイルのフラグ (映像/音声の有無を示すビットフラグ)
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
  int inter;      //	フレームのインターレース種別 (標準/反転/奇数/偶数/二重化/自動)
  int index24fps; //	現在は使用されていません
  int config;     //	フレームのプロファイル環境の番号
  int vcm;        //	フレームの圧縮設定の番号
  int edit_flag;  //	編集フラグ (キーフレーム/マークフレーム/優先間引き/コピーフレームを示すビットフラグ)
  int reserve[9]; //	拡張用に予約されてます
};

} // namespace mu
