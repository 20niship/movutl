#include <filesystem>
#include <string>

namespace sfs = std::filesystem;

namespace mu {
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
