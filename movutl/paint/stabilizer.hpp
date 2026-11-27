#pragma once

#include <movutl/paint/brush.hpp>
#include <vector>

namespace mu {

// 手ブレ補正: 直近の入力位置の移動平均。筆圧は素通し。入力1点につき必ず1点を返す
class Stabilizer {
public:
  explicit Stabilizer(int strength = 0); // 0〜100。0は素通し
  PenSample add(const PenSample& p);     // 最初の点はそのまま返る
  std::vector<PenSample> finish();       // ペンを上げた時: 最後の入力位置へ追いつくまでの点(最後は入力位置と一致)

private:
  // ponytail: 窓はサンプル数なので入力の頻度が低い(マウス)ほど遅れが大きい。気になれば時間か距離で窓を切る
  size_t window_;
  std::vector<PenSample> history_; // 直近window_個の入力
};

} // namespace mu
