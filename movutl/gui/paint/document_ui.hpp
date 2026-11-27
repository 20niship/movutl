#pragma once

namespace mu {

// 毎フレーム、ウィンドウの外(メニューバーの後)で呼ぶ
void draw_document_dialogs();
// 終了してよければtrue。未保存の文書があれば確認ダイアログを出してfalseを返し、続行が選ばれたら終了し直す
bool confirm_quit();

namespace detail {
void register_document_commands(); // キャンバスの新規/開く/保存と、動画編集の新規プロジェクト
}

} // namespace mu
