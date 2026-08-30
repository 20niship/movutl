#include <doctest/doctest.h>
#include <movutl/app/export_state.hpp>

using namespace mu;

TEST_CASE("ExportProgress: is_exporting()はrunningフラグを反映する") {
  auto& prog = get_export_progress();
  prog.running.store(false);
  CHECK_FALSE(is_exporting());
  prog.running.store(true);
  CHECK(is_exporting());
  prog.running.store(false);
  CHECK_FALSE(is_exporting());
}

TEST_CASE("ExportProgress: request_cancel_export()はcancel_requestedを立てる") {
  auto& prog = get_export_progress();
  prog.cancel_requested.store(false);
  CHECK_FALSE(prog.cancel_requested.load());
  request_cancel_export();
  CHECK(prog.cancel_requested.load());
  prog.cancel_requested.store(false); // 他テストへ影響しないよう後始末
}
