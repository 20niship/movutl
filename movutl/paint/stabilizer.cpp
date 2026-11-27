#include <algorithm>
#include <movutl/paint/stabilizer.hpp>

namespace mu {

Stabilizer::Stabilizer(int strength) : window_(1 + std::clamp(strength, 0, 100) / 2) {}

PenSample Stabilizer::add(const PenSample& p) {
  if(history_.empty())
    history_.assign(window_, p);
  else {
    history_.erase(history_.begin());
    history_.push_back(p);
  }
  float x = 0, y = 0; // 現在値との差を平均する(同じ点が並べば誤差なく一致する)
  for(const auto& h : history_) {
    x += h.x - p.x;
    y += h.y - p.y;
  }
  return {p.x + x / window_, p.y + y / window_, p.pressure};
}

std::vector<PenSample> Stabilizer::finish() {
  std::vector<PenSample> v;
  if(history_.empty()) return v;
  const PenSample end = history_.back();
  // 終点を入れ続けて窓を埋める。すでに追いついていれば何も足さない
  while(std::any_of(history_.begin(), history_.end(), [&](const PenSample& h) { return h.x != end.x || h.y != end.y; })) v.push_back(add(end));
  history_.clear();
  return v;
}

} // namespace mu
