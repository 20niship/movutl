#include <movutl/command/exo/exo_report.hpp>

namespace mu {

namespace {
ExoImportReport g_report;
}

void ExoImportReport::add(const std::string& msg) {
  for(auto& it : items) {
    if(it.msg == msg) {
      ++it.count;
      return;
    }
  }
  items.push_back({msg, 1});
}

ExoImportReport& exo_import_report() { return g_report; }

void exo_import_report_begin(const std::string& path) {
  g_report      = ExoImportReport();
  g_report.path = path;
}

} // namespace mu
