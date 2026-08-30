#include <movutl/app/export_state.hpp>

namespace mu {

ExportProgress& get_export_progress() {
  static ExportProgress progress;
  return progress;
}

bool is_exporting() { return get_export_progress().running.load(); }

void request_cancel_export() { get_export_progress().cancel_requested.store(true); }

} // namespace mu
