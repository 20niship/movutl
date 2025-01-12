#pragma once
#include <chrono>
#include <movutl/core/logger.hpp>

namespace mu {

struct ScopeTimer {
  ScopeTimer(const char* name) : name(name), start(std::chrono::high_resolution_clock::now()) {}
  ~ScopeTimer() {
    end = std::chrono::high_resolution_clock::now();
    auto dur = end - start;
    using namespace std::chrono;
    auto d_us = duration_cast<microseconds>(dur).count();
    if(d_us < 1000) {
      LOG_F(1, "%s: %d us", name, (int)d_us);
      return;
    }

    auto d_ms = duration_cast<milliseconds>(dur).count();
    if(d_ms < 1000) {
      LOG_F(1, "%s: %d ms", name, (int)d_ms);
      return;
    }

    auto d_s = duration_cast<seconds>(dur).count();
    LOG_F(1, "%s: %d s", name, (int)d_s);
  }

  const char* name;
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
};


} // namespace mu
