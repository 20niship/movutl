#pragma once
#include <vector>
//
#include <cutil/string.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

//  システムフラグ (旧 SYS_INFO::flag)
constexpr int kSysFlagEdit = 1; // 編集中

struct Config {
public:
  MOVUTL_DECLARE_SINGLETON(Config);
  Config()  = default;
  ~Config() = default;

  Vec2d max_size   = {3000, 3000}; // 編集出来る最大画像サイズ
  int max_frame    = 320000;       // 編集出来る最大フレーム数
  int cache_frames = 10;
  std::vector<std::string> plugin_search_paths = {"plugins"};
  bool log_to_file         = false;
  std::string log_filename = "log.txt";
  LogLevel log_level       = LogLevel::DEBUG;

  // 以下 movutl/core/sys_info.hpp より統合
  int sys_flag = 0;                  // システムフラグ (kSysFlagEdit 等)
  cutil::Str version_info{""};       // バージョン情報
  int filter_count   = 0;            // 登録されてるフィルタの数
  Vec2d min_size     = {1, 1};       // 編集出来る最小画像サイズ
  Vec2d vram_size    = {3000, 3000}; // 編集用画像領域のサイズ
  int vram_yc_size   = 6;            // 編集用画像領域の画素のバイト数 (YC48想定)
  int vram_line_size = 0;            // 編集用画像領域の幅のバイト数
  void* font_handle  = nullptr;      // フィルタ設定ウィンドウで使用しているフォントのハンドル (Windows HFONT)
  int build_number   = 0;            // ビルド番号 (新しいバージョンになるほど大きな値になります)

  static void Load();
  static void Save();
  static void Reload();
};
} // namespace mu
