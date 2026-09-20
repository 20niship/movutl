#pragma once

#include <filesystem>
#include <string>

namespace mu {

// CP932(Shift_JIS)をUTF-8へ変換する。不正バイトは読み飛ばす
std::string cp932_to_utf8(const std::string& src);

// UTF-8かどうかを検証する(BOMは許容)
bool is_valid_utf8(const std::string& s);

// テキストのエンコードを推定してUTF-8へ揃える。BOM除去、妥当なUTF-8ならそのまま、それ以外はCP932とみなす
std::string to_utf8_guess(const std::string& raw);

// ファイルを読み込みto_utf8_guessでUTF-8化して返す。開けなければfalse
bool read_text_file_utf8(const std::filesystem::path& path, std::string& out);

} // namespace mu
