#pragma once

#include <movutl/asset/image.hpp>
#include <mutex>
#include <unordered_map>

namespace mu {

// Composition1個分のレンダリング済みフレームキャッシュ。スレッド安全。
class FrameCache {
public:
  bool get(int frame, Ref<Image>* out) const;
  void insert(int frame, Ref<Image> img, int current_frame);
  bool is_cached(int frame) const;
  size_t size() const;

  void invalidate_all();
  void invalidate_range(int f0, int f1);

private:
  mutable std::mutex mtx_;
  std::unordered_map<int, Ref<Image>> frames_;

  void evict_locked(int current_frame); // 保持上限はConfig::Get()->cache_framesを参照する
};

} // namespace mu
