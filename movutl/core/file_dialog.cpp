#include <movutl/core/filesystem.hpp>
#include <movutl/core/platform.hpp>

#ifdef MOVUTL_PLATFORM_WINDOWS
#include <commdlg.h>
#include <sstream>
#include <string>
#include <vector>
#include <windows.h>

namespace mu {

std::string select_file_dialog(const std::string& title, const std::vector<std::string>& extensions) {
  OPENFILENAME ofn;
  char szFile[MAX_PATH] = {0};
  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner   = nullptr;
  ofn.lpstrFile   = szFile;
  ofn.nMaxFile    = sizeof(szFile);

  std::ostringstream filter;
  for(const auto& ext : extensions) {
    filter << "*." << ext << "\0";
  }
  filter << "\0";
  std::string filterStr = filter.str();
  ofn.lpstrFilter       = filterStr.c_str();
  ofn.nFilterIndex      = 1;
  ofn.Flags             = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

  if(GetOpenFileName(&ofn) == TRUE) {
    return std::string(szFile);
  }
  return "";
}
} // namespace mu

#elif !defined(MOVUTL_PLATFORM_MACOS)
#include <movutl/core/logger.hpp>

namespace mu {
// ponytail: Linux向けネイティブファイルダイアログは未実装、GTK/zenity等を使う実装に置き換え可能
std::string select_file_dialog(const std::string& title, const std::vector<std::string>& extensions) {
  (void)title;
  (void)extensions;
  LOG_F(WARNING, "select_file_dialog: not implemented on this platform");
  return "";
}
} // namespace mu
#endif
