#include <cstdio>
#include <filesystem>
#include <functional>
#include <movutl/core/assert.hpp>
#include <string>

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

bool fs_exists(const std::string& path) { return sfs::exists(path); }

bool fs_is_directory(const std::string& path) { return sfs::is_directory(path); }

bool fs_is_file(const std::string& path) { return sfs::is_regular_file(path); }

bool fs_create_directory(const std::string& path) { return sfs::create_directory(path); }

std::string fs_extension(const std::string& path) {
  auto ext = sfs::path(path).extension().string();
  if(!ext.empty() && ext[0] == '.') ext = ext.substr(1);
  return ext;
}

int64_t fs_last_write_time_raw(const std::string& path) {
  std::error_code ec;
  auto t = sfs::last_write_time(path, ec);
  if(ec) return 0;
  return (int64_t)t.time_since_epoch().count();
}

std::string fs_cache_key(const std::string& path, int64_t mtime) {
  std::hash<std::string> h;
  size_t v = h(path + "|" + std::to_string(mtime));
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%016zx", v);
  return buf;
}

std::vector<std::string> get_available_fonts() {
  std::vector<std::string> fonts;

#if defined(__APPLE__)
  const std::string dir = "/System/Library/Fonts";
#elif defined(__linux__)
  const std::string dir = "/usr/share/fonts";
#elif defined(_WIN32)
  const std::string dir = "C:/Windows/Fonts";
#else
  return fonts;
#endif

  if(!fs_exists(dir)) return fonts;

  std::error_code ec;
  for(auto it = sfs::recursive_directory_iterator(dir, sfs::directory_options::skip_permission_denied, ec); !ec && it != sfs::recursive_directory_iterator(); it.increment(ec)) {
    if(!it->is_regular_file(ec) || ec) continue;
    const auto ext = fs_extension(it->path().string());
    if(ext == "ttf" || ext == "otf" || ext == "ttc") {
      fonts.push_back(it->path().string());
    }
  }
  return fonts;
}

} // namespace mu
