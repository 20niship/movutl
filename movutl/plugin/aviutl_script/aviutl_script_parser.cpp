#include <movutl/core/logger.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <regex>
#include <sstream>

namespace mu::detail {

namespace {
std::string trim(const std::string& s) {
  size_t a = s.find_first_not_of(" \t\r\n");
  if(a == std::string::npos) return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, ',')) out.push_back(trim(item));
  return out;
}

float parse_float_or(const std::string& s, float def) {
  try {
    return std::stof(s);
  } catch(...) {
    return def;
  }
}
} // namespace

std::vector<AviUtlScriptDef> parse_aviutl_script(const std::string& text) {
  static const std::regex track_re(R"(^--track(\d):(.*)$)");
  static const std::regex check_re(R"(^--check(\d):(.*)$)");
  static const std::regex at_re(R"(^@(.*)$)");

  std::vector<AviUtlScriptDef> result;
  std::vector<AviUtlTrackDef> pending_tracks;
  std::vector<AviUtlCheckDef> pending_checks;
  std::string pending_dialog_code;
  std::vector<std::string> body_lines;
  std::string current_name; // 空文字は「@未指定、ファイル全体が単一スクリプト」を意味する(AviUtl仕様)
  bool seen_at = false;

  auto push_current_block = [&]() {
    AviUtlScriptDef def;
    def.name        = current_name;
    def.tracks      = pending_tracks;
    def.checks      = pending_checks;
    def.dialog_code = pending_dialog_code;
    std::ostringstream body;
    for(auto& l : body_lines) body << l << "\n";
    def.lua_body = body.str();
    result.push_back(std::move(def));
    pending_tracks.clear();
    pending_checks.clear();
    pending_dialog_code.clear();
    body_lines.clear();
  };

  // `@`出現前のbody_linesは冒頭コメント/説明文でしかありえない(AviUtl仕様上コード扱いされない)ため破棄しtrack/checkのみ持ち越す
  auto flush_on_at = [&]() {
    if(seen_at)
      push_current_block();
    else
      body_lines.clear();
  };

  std::istringstream stream(text);
  std::string line;
  std::smatch m;
  while(std::getline(stream, line)) {
    if(!line.empty() && line.back() == '\r') line.pop_back();

    if(std::regex_match(line, m, track_re)) {
      auto parts = split_csv(m[2].str());
      AviUtlTrackDef t;
      if(parts.size() > 0) t.name = parts[0];
      if(parts.size() > 1) t.min_value = parse_float_or(parts[1], 0.0f);
      if(parts.size() > 2) t.max_value = parse_float_or(parts[2], 100.0f);
      if(parts.size() > 3) t.default_value = parse_float_or(parts[3], 0.0f);
      if(parts.size() > 4) t.step = parse_float_or(parts[4], 1.0f);
      pending_tracks.push_back(t);
      continue;
    }
    if(std::regex_match(line, m, check_re)) {
      auto parts = split_csv(m[2].str());
      AviUtlCheckDef c;
      if(parts.size() > 0) c.name = parts[0];
      if(parts.size() > 1) c.default_value = parse_float_or(parts[1], 0.0f) != 0.0f;
      pending_checks.push_back(c);
      continue;
    }
    if(line.rfind("--dialog:", 0) == 0) {
      // UIウィジェット自体は非対応。各項目の「項目名,変数初期化コード」からコード片だけ抽出し、後でlua_bodyの前に結合して実行する
      std::string rest = line.substr(9);
      std::stringstream ss(rest);
      std::string item;
      while(std::getline(ss, item, ';')) {
        size_t comma = item.find(',');
        if(comma == std::string::npos) continue;
        std::string code = trim(item.substr(comma + 1));
        if(!code.empty()) pending_dialog_code += code + "\n";
      }
      continue;
    }

    if(std::regex_match(line, m, at_re)) {
      flush_on_at();
      current_name = trim(m[1].str());
      seen_at      = true;
      continue;
    }

    body_lines.push_back(line);
  }

  // ファイル末尾: 蓄積中のtrack/check/bodyを最後のブロックとして確定する(@が一度もなければファイル全体が単一スクリプトになる)
  bool has_body = false;
  for(auto& l : body_lines)
    if(!trim(l).empty()) {
      has_body = true;
      break;
    }
  if(has_body || !pending_tracks.empty() || !pending_checks.empty()) push_current_block();

  if(result.empty()) LOG_F(WARNING, "parse_aviutl_script: 有効なスクリプトブロックが見つかりませんでした");
  return result;
}

} // namespace mu::detail
