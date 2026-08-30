#pragma once

#include <map>
#include <mutex>
#include <movutl/asset/image.hpp>

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
  std::map<int, Ref<Image>> frames_;
  static constexpr size_t kMaxCachedFrames = 300;

  void evict_locked(int current_frame);
};

} // namespace mu
