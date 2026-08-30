#include <cstdlib>
#include <movutl/render2d/frame_cache.hpp>

namespace mu {

bool FrameCache::get(int frame, Ref<Image>* out) const {
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = frames_.find(frame);
  if(it == frames_.end()) return false;
  if(out) *out = it->second;
  return true;
}

void FrameCache::insert(int frame, Ref<Image> img, int current_frame) {
  std::lock_guard<std::mutex> lock(mtx_);
  frames_[frame] = std::move(img);
  evict_locked(current_frame);
}

bool FrameCache::is_cached(int frame) const {
  std::lock_guard<std::mutex> lock(mtx_);
  return frames_.count(frame) != 0;
}

size_t FrameCache::size() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return frames_.size();
}

void FrameCache::invalidate_all() {
  std::lock_guard<std::mutex> lock(mtx_);
  frames_.clear();
}

void FrameCache::invalidate_range(int f0, int f1) {
  if(f0 > f1) std::swap(f0, f1);
  std::lock_guard<std::mutex> lock(mtx_);
  frames_.erase(frames_.lower_bound(f0), frames_.upper_bound(f1));
}

void FrameCache::evict_locked(int current_frame) {
  while(frames_.size() > kMaxCachedFrames) {
    auto farthest = frames_.begin();
    int farthest_dist = std::abs(farthest->first - current_frame);
    for(auto it = frames_.begin(); it != frames_.end(); ++it) {
      int dist = std::abs(it->first - current_frame);
      if(dist > farthest_dist) {
        farthest      = it;
        farthest_dist = dist;
      }
    }
    frames_.erase(farthest);
  }
}

} // namespace mu
