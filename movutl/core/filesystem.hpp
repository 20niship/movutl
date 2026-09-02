#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace mu {

std::string fs_get_asset_path();
std::string fs_get_font_path();

bool fs_exists(const std::string& path);
bool fs_is_directory(const std::string& path);
bool fs_is_file(const std::string& path);
bool fs_create_directory(const std::string& path);
std::string fs_extension(const std::string& path);
int64_t fs_last_write_time_raw(const std::string& path);          // last_write_timeのtime_since_epoch().count()。取得失敗時は0
std::string fs_cache_key(const std::string& path, int64_t mtime); // path+mtimeから16進ハッシュ文字列を生成する

std::string select_file_dialog(const std::string& title, const std::vector<std::string>& extensions);
// 保存先を選択するOSネイティブダイアログ(select_file_os_dialog)。default_nameは初期表示するファイル名
std::string select_save_file_dialog(const std::string& title, const std::string& default_name, const std::vector<std::string>& extensions);
std::vector<std::string> get_available_fonts();
} // namespace mu
