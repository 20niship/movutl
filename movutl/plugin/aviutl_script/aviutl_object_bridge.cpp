#include <filesystem>
#include <fstream>
#include <movutl/asset/config.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <movutl/plugin/plugin.hpp>
#include <sstream>

namespace mu::detail {

namespace {

void register_custom_objects_from_file(const std::filesystem::path& path) {
  std::ifstream ifs(path);
  if(!ifs) {
    LOG_F(ERROR, "register_custom_objects: ファイルを開けません: %s", path.string().c_str());
    return;
  }
  std::ostringstream ss;
  ss << ifs.rdbuf();
  auto defs = parse_aviutl_script(ss.str());
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
  for(const auto& dir : Config::Get()->lua_script_dirs) {
    if(!fs::exists(dir) || !fs::is_directory(dir)) continue;
    for(const auto& entry : fs::recursive_directory_iterator(dir)) {
      if(!entry.is_regular_file() || entry.path().extension() != ".obj") continue;
      LOG_F(1, "Loading custom object script: %s", entry.path().string().c_str());
      register_custom_objects_from_file(entry.path());
    }
  }
}

} // namespace mu::detail
