#pragma once
#include <string>

namespace mu {

// AviUtl拡張編集の.exoを読み込み、含まれる全オブジェクトをアクティブCompositionのレイヤーへ追加する
// (通常の入力プラグインは1ファイル=1Entityだが、こちらはプロジェクト単位で複数トラックを追加する)
// 追加したEntity数を返す。ファイルが開けない場合は-1
int import_exo_file(const char* path);

// import_exoコマンド(拡張子exoに紐付く)を登録する
void register_exo_command();

namespace detail {
// テスト用に公開する変換関数
std::string exo_cp932_to_utf8(const std::string& src);
std::string exo_utf16le_hex_to_utf8(const std::string& hex);
} // namespace detail

} // namespace mu
