#pragma once

#include <movutl/paint/brush.hpp>
#include <vector>

namespace mu {

enum class PenPhase { Down, Move, Up };

// ペン/マウスの入力1件
struct PenEvent {
  PenSample sample; // 座標系は入力元のまま(ツールへ渡す前にキャンバス座標へ変換する)
  PenPhase phase = PenPhase::Move;
  bool eraser    = false; // ペンの消しゴム側
  double time    = 0;     // 秒
};

// 入力デバイス。毎フレームpoll()して、前回以降のイベントを順番通りに全部受け取る
class PenInput {
public:
  virtual ~PenInput()                  = default;
  virtual std::vector<PenEvent> poll() = 0;
};

} // namespace mu
