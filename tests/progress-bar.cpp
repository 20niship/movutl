#include <ext/pbar/pbar.hpp>

#include <chrono>
#include <doctest/doctest.h>
#include <iostream>
#include <thread>

TEST_CASE("progress-bar") {
  using namespace std::this_thread;
  using namespace std::chrono;

  constexpr auto total_ = 30;
  constexpr auto ncols = 100;
  constexpr auto description = "[TASK0]";
  pbar::pbar bar(total_, ncols, description);
  bar.enable_recalc_console_width(1); // check console width every tick
  bar.disable_time_measurement();
  bar.init(); // show a bar with zero progress

  for(auto i = 0; i < total_; ++i, ++bar) {
    sleep_for(milliseconds(20));
  }
}
