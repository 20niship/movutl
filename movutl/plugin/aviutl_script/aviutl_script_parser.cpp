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
  std::vector<std::string> body_lines;
  std::string current_name;
  bool in_block = false;

  auto flush = [&]() {
    if(!in_block) return;
    AviUtlScriptDef def;
    def.name   = current_name;
    def.tracks = pending_tracks;
    def.checks = pending_checks;
    std::ostringstream body;
    for(auto& l : body_lines) body << l << "\n";
    def.lua_body = body.str();
    result.push_back(std::move(def));
    pending_tracks.clear();
    pending_checks.clear();
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
    if(line.rfind("--dialog:", 0) == 0) continue; // ダイアログUIは今回未対応

    if(std::regex_match(line, m, at_re)) {
      flush();
      current_name = trim(m[1].str());
      in_block     = true;
      continue;
    }

    if(in_block) body_lines.push_back(line);
  }
  flush();

  if(result.empty()) LOG_F(WARNING, "parse_aviutl_script: '@'ブロックが見つかりませんでした");
  return result;
}

} // namespace mu::detail
