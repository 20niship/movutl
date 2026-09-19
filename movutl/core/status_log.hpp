#pragma once
#include <string>
#include <vector>

namespace mu {

// ステータスバーに出すユーザー向けの短い操作ログ(LOG_Fとは別)。どのスレッドからでもpushできる
enum class StatusLevel { Info, Success, Warning, Error };

struct StatusLogEntry {
  StatusLevel level = StatusLevel::Info;
  std::string message;
  double time_sec = 0; // アプリ起動からの経過秒(フェード判定用)
  std::string clock;   // 履歴表示用の時刻(HH:MM:SS)
};

void push_status_log(StatusLevel level, const std::string& message);
bool status_log_latest(StatusLogEntry* out);
std::vector<StatusLogEntry> status_log_history(); // 古い順、最大100件
double status_log_now_sec();

// 未保存の変更があるか。厳密な変更検出ではなく、コマンド実行/読み込み/保存の経路から更新するUI表示用の近似
void status_log_set_dirty(bool dirty);
bool status_log_dirty();

} // namespace mu
