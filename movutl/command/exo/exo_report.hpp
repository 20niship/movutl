#pragma once
#include <string>
#include <vector>

namespace mu {

// exo取り込みで反映できなかった項目(未対応オブジェクト/エフェクト/アニメーション等)の直近1回分の一覧
struct ExoImportReport {
  struct Item {
    std::string msg;
    int count = 1; // 同じ内容の重複件数
  };
  std::string path;
  int imported = 0;
  std::vector<Item> items;

  void add(const std::string& msg);
};

ExoImportReport& exo_import_report();
// 取り込み開始時に呼び、前回の結果を破棄する
void exo_import_report_begin(const std::string& path);

// 未対応項目がある場合に取り込み結果ダイアログを次のフレームで開く
void exo_import_report_request_dialog();
// メインメニュー描画後にGUIスレッドから毎フレーム呼ぶ(モーダルで直近の取り込み結果を表示する)
void draw_exo_import_report_dialog();

} // namespace mu
