#pragma once
#include <atomic>

namespace mu {

struct ExportProgress {
  std::atomic<bool> running{false};
  std::atomic<bool> cancel_requested{false};
  std::atomic<int> current_frame{0};
  std::atomic<int> total_frames{0};
};

ExportProgress& get_export_progress();
bool is_exporting();
void request_cancel_export();

} // namespace mu
