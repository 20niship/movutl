#pragma once

#include <movutl/input/pen_input.hpp>
#include <utility>

namespace mu {

// マウスをペンとして扱う(筆圧は常に1)。GUIが毎フレームupdate()で座標とボタン状態を渡す
class MousePenInput : public PenInput {
public:
  void update(float x, float y, bool down, double time) {
    const bool moved = x != x_ || y != y_;
    x_               = x;
    y_               = y;
    if(down && !down_)
      push(PenPhase::Down, time);
    else if(!down && down_)
      push(PenPhase::Up, time);
    else if(down && moved)
      push(PenPhase::Move, time);
    down_ = down;
  }
  std::vector<PenEvent> poll() override { return std::exchange(events_, {}); }

private:
  void push(PenPhase phase, double time) { events_.push_back({{x_, y_, 1.0f}, phase, false, time}); }
  float x_ = 0, y_ = 0;
  bool down_ = false;
  std::vector<PenEvent> events_;
};

} // namespace mu
