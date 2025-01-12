#pragma once
#include <string>

namespace mu {

std::string fs_get_asset_path();
std::string fs_get_font_path();

bool fs_exists(const std::string& path);
bool fs_is_directory(const std::string& path);
bool fs_is_file(const std::string& path);
bool fs_create_directory(const std::string& path);
std::string fs_extension(const std::string& path);

} // namespace mu
