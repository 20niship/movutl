#include <atomic>
#include <chrono>
#include <ctime>
#include <deque>
#include <movutl/core/status_log.hpp>
#include <mutex>

namespace mu {

namespace {
constexpr size_t kMaxHistory = 100;
std::mutex g_mtx;
std::deque<StatusLogEntry> g_history;
std::atomic<bool> g_dirty{false};
const auto g_start = std::chrono::steady_clock::now();
} // namespace

double status_log_now_sec() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - g_start).count(); }

void push_status_log(StatusLevel level, const std::string& message) {
  StatusLogEntry e;
  e.level       = level;
  e.message     = message;
  e.time_sec    = status_log_now_sec();
  std::time_t t = std::time(nullptr);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&t));
  e.clock = buf;
  std::lock_guard<std::mutex> lock(g_mtx);
  if(!g_history.empty() && g_history.back().message == e.message && g_history.back().level == e.level) {
    g_history.back().time_sec = e.time_sec; // 同一メッセージの連続は1件にまとめる(毎フレームの警告等で履歴が埋まらないように)
    g_history.back().clock    = e.clock;
    return;
  }
  g_history.push_back(std::move(e));
  if(g_history.size() > kMaxHistory) g_history.pop_front();
}

bool status_log_latest(StatusLogEntry* out) {
  std::lock_guard<std::mutex> lock(g_mtx);
  if(g_history.empty()) return false;
  *out = g_history.back();
  return true;
}

std::vector<StatusLogEntry> status_log_history() {
  std::lock_guard<std::mutex> lock(g_mtx);
  return {g_history.begin(), g_history.end()};
}

void status_log_set_dirty(bool dirty) { g_dirty = dirty; }
bool status_log_dirty() { return g_dirty; }

} // namespace mu
