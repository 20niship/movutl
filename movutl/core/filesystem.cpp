#include <filesystem>
#include <string>
#include <movutl/core/assert.hpp>

namespace sfs = std::filesystem;

namespace mu {
/**************************************************
   --- resource handling ---
**************************************************/

std::string fs_get_asset_path() {
  static std::string asset_path;
  if(!asset_path.empty()) return asset_path;

  const char* requirements[] = {
    "fonts/fa-sold-900.ttf",
    "fonts/fa-sold-900.ttf",
    "fonts/Meiryo.ttf",
  };
  const char* candidates[] = {
    "assets",
    "../assets",
    "../../assets",
    "../../../assets",
  };
  for(auto& candidate : candidates) {
    for(auto& req : requirements) {
      auto path = sfs::path(candidate).append(req);
      if(sfs::exists(path)) {
        asset_path = candidate;
        return asset_path;
      }
    }
  }
  MU_FAIL("fs_get_asset_path: could not find assets directory");
  return "";
}

std::string fs_get_font_path() {
  auto as = fs_get_asset_path();
  return sfs::path(as).append("fonts").string();
}


/**************************************************
 * thin std::filesystem wrapper (for faster compile times)
 **************************************************/

bool fs_exists(const std::string& path) {
  return sfs::exists(path);
}

bool fs_is_directory(const std::string& path) {
  return sfs::is_directory(path);
}

bool fs_is_file(const std::string& path) {
  return sfs::is_regular_file(path);
}

bool fs_create_directory(const std::string& path) {
  return sfs::create_directory(path);
}

std::string fs_extension(const std::string& path) {
  auto ext = sfs::path(path).extension().string();
  if(!ext.empty() && ext[0] == '.') ext = ext.substr(1);
  return ext;
}

} // namespace mu
