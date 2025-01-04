#pragma once
#include <string>

namespace mu {
//	システムインフォメーション構造体
typedef struct {
  int flag;                  //	システムフラグ
                             //	SYS_INFO_FLAG_EDIT		: 編集中
                             //	SYS_INFO_FLAG_VFAPI		: VFAPI動作時
                             //	SYS_INFO_FLAG_USE_SSE	: SSE使用
                             //	SYS_INFO_FLAG_USE_SSE2	: SSE2使用
  std::string info;          //	バージョン情報
  int filter_n;              //	登録されてるフィルタの数
  int min_w, min_h;          //	編集出来る最小画像サイズ
  int max_w, max_h;          //	編集出来る最大画像サイズ
  int max_frame;             //	編集出来る最大フレーム数
  int vram_w, vram_h;        //	編集用画像領域のサイズ
  int vram_yc_size;          //	編集用画像領域の画素のバイト数
  int vram_line_size;        //	編集用画像領域の幅のバイト数
  HFONT hfont;               //	フィルタ設定ウィンドウで使用しているフォントのハンル
  int build;                 //	ビルド番号 (新しいバージョンになるほど大きな値になります)
  int reserve[2];            //	拡張用に予約されてます
} SYS_INFO;
#define SYS_INFO_FLAG_EDIT 1
} // namespace mu
