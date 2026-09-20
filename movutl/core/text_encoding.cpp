#include <cerrno>
#include <fstream>
#include <movutl/core/text_encoding.hpp>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#else
#include <iconv.h>
#endif

namespace mu {

std::string cp932_to_utf8(const std::string& src) {
  if(src.empty()) return {};
#ifdef _WIN32
  int wn = MultiByteToWideChar(932, 0, src.data(), (int)src.size(), nullptr, 0);
  if(wn <= 0) return src;
  std::wstring w(wn, L'\0');
  MultiByteToWideChar(932, 0, src.data(), (int)src.size(), w.data(), wn);
  int un = WideCharToMultiByte(CP_UTF8, 0, w.data(), wn, nullptr, 0, nullptr, nullptr);
  std::string out(un, '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.data(), wn, out.data(), un, nullptr, nullptr);
  return out;
#else
  iconv_t cd = iconv_open("UTF-8", "CP932");
  if(cd == (iconv_t)-1) return src;
  std::string out(src.size() * 4, '\0');
  char* in       = const_cast<char*>(src.data());
  size_t in_left = src.size();
  char* op       = out.data();
  size_t o_left  = out.size();
  // 不正バイトは読み飛ばして変換を続ける
  while(in_left > 0) {
    if(iconv(cd, &in, &in_left, &op, &o_left) == (size_t)-1) {
      if(errno == EILSEQ || errno == EINVAL) {
        ++in;
        --in_left;
        continue;
      }
      break;
    }
  }
  iconv_close(cd);
  out.resize(op - out.data());
  return out;
#endif
}

bool is_valid_utf8(const std::string& s) {
  size_t i = 0, n = s.size();
  while(i < n) {
    unsigned char c = s[i];
    int len         = c < 0x80 ? 1 : (c >> 5) == 0x6 ? 2 : (c >> 4) == 0xE ? 3 : (c >> 3) == 0x1E ? 4 : 0;
    if(len == 0 || i + len > n) return false;
    if(len == 2 && c < 0xC2) return false; // 過長表現
    for(int k = 1; k < len; ++k)
      if(((unsigned char)s[i + k] >> 6) != 0x2) return false;
    i += len;
  }
  return true;
}

std::string to_utf8_guess(const std::string& raw) {
  if(raw.compare(0, 3, "\xEF\xBB\xBF") == 0) return raw.substr(3);
  if(is_valid_utf8(raw)) return raw; // ASCIIのみもここ。CP932の日本語が偶然妥当なUTF-8になる確率は極小
  return cp932_to_utf8(raw);
}

bool read_text_file_utf8(const std::filesystem::path& path, std::string& out) {
  std::ifstream ifs(path, std::ios::binary);
  if(!ifs) return false;
  std::ostringstream ss;
  ss << ifs.rdbuf();
  out = to_utf8_guess(ss.str());
  return true;
}

} // namespace mu
