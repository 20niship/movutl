#include <filesystem>
#include <fstream>
#include <movutl/asset/config.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/text_encoding.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <movutl/plugin/plugin.hpp>
#include <set>
#include <sstream>

namespace mu::detail {

namespace {

void register_custom_objects_from_file(const std::filesystem::path& path) {
  std::string text;
  if(!read_text_file_utf8(path, text)) {
    LOG_F(ERROR, "register_custom_objects: ファイルを開けません: %s", path.string().c_str());
    return;
  }
  auto defs = parse_aviutl_script(text);
  if(defs.empty()) return;
  std::string stem = path.stem().string();
  for(auto& def : defs) {
    if(def.name.empty()) def.name = stem; // `@名前`ブロックが無いファイルはファイル名をオブジェクト名とする単一スクリプト扱いになる(AviUtl仕様)
    if(!def.dialog_code.empty()) def.lua_body = def.dialog_code + def.lua_body;
    CustomObjectRegistry::Get()->register_object(std::make_unique<AviUtlScriptDef>(std::move(def)));
  }
}

} // namespace

void register_custom_objects() {
  namespace fs = std::filesystem;
  // lua_script_dirsはビルドフォルダ内のコピーとリポジトリ直下の両方を指しうるので、フォルダからの相対パスが同じファイルは最初に見つかった方だけ登録する(二重登録の防止)
  std::set<fs::path> seen;
  for(const auto& dir : Config::Get()->lua_script_dirs) {
    if(!fs::exists(dir) || !fs::is_directory(dir)) continue;
    for(const auto& entry : fs::recursive_directory_iterator(dir)) {
      if(!entry.is_regular_file() || entry.path().extension() != ".obj") continue;
      if(!seen.insert(fs::relative(entry.path(), dir)).second) continue;
      LOG_F(1, "Loading custom object script: %s", entry.path().string().c_str());
      register_custom_objects_from_file(entry.path());
    }
  }
}

} // namespace mu::detail
