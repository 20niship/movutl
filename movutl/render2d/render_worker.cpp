#include <algorithm>
#include <movutl/asset/config.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/render2d/render_worker.hpp>
#include <movutl/render2d/renderer.hpp>

namespace mu {

size_t RenderWorkerPool::default_worker_count() {
  auto n = std::thread::hardware_concurrency();
  if(n == 0) n = 2;
  return std::min<size_t>(4, n);
}

RenderWorkerPool::RenderWorkerPool(size_t n_workers) {
  worker_frame_ = std::make_unique<std::atomic<int>[]>(n_workers);
  for(size_t i = 0; i < n_workers; i++) worker_frame_[i].store(-1);
  for(size_t i = 0; i < n_workers; i++) workers_.emplace_back([this, i] { worker_loop(i); });
}

RenderWorkerPool::~RenderWorkerPool() { stop(); }

void RenderWorkerPool::request(Composition* comp, int frame, bool urgent) {
  if(!comp) return;
  std::lock_guard<std::mutex> lock(qmtx_);
  Job job{comp, frame};
  if(pending_.count(job)) return;
  pending_.insert(job);
  if(urgent)
    queue_.push_front(job);
  else
    queue_.push_back(job);
  cv_.notify_one();
}

void RenderWorkerPool::tick(Composition* comp, bool playing) {
  if(!comp) return;
  int cur;
  {
    std::lock_guard<std::mutex> lock(comp->mtx);
    cur = comp->frame;
  }
  if(!comp->cache.is_cached(cur)) request(comp, cur, true);

  // 空きキャッシュ枠分だけ先読みする(停止中は再生中より控えめに)
  int capacity  = std::max(1, Config::Get()->cache_frames);
  int remaining = capacity - (int)comp->cache.size();
  if(remaining <= 0) return;
  int ahead = playing ? remaining : std::min(remaining, 10);
  for(int i = 1; i <= ahead; i++) {
    int f = cur + i;
    if(f > comp->fend) break;
    if(!comp->cache.is_cached(f)) request(comp, f, false);
  }
}

void RenderWorkerPool::stop() {
  if(stop_.exchange(true)) return;
  cv_.notify_all();
  for(auto& t : workers_)
    if(t.joinable()) t.join();
  workers_.clear();
}

std::vector<RenderWorkerPool::WorkerStatus> RenderWorkerPool::worker_status() const {
  std::vector<WorkerStatus> out(workers_.size());
  for(size_t i = 0; i < workers_.size(); i++) {
    int f  = worker_frame_[i].load();
    out[i] = {f, f >= 0};
  }
  return out;
}

size_t RenderWorkerPool::queue_size() const {
  std::lock_guard<std::mutex> lock(qmtx_);
  return queue_.size();
}

void RenderWorkerPool::worker_loop(size_t worker_idx) {
  CPURenderer renderer;
  while(true) {
    Job job{nullptr, 0};
    {
      std::unique_lock<std::mutex> lock(qmtx_);
      cv_.wait(lock, [this] { return stop_.load() || !queue_.empty(); });
      if(stop_.load() && queue_.empty()) return;
      job = queue_.front();
      queue_.pop_front();
    }
    worker_frame_[worker_idx].store(job.frame);
    Ref<Image> out;
    int cur_frame_for_evict;
    {
      std::lock_guard<std::mutex> lock(job.comp->mtx);
      cur_frame_for_evict = job.comp->frame;
    }
    if(renderer.render_frame(job.comp, job.frame, out)) {
      job.comp->cache.insert(job.frame, out, cur_frame_for_evict);
      total_rendered_.fetch_add(1);
    }
    worker_frame_[worker_idx].store(-1);
    {
      // cache挿入後にpending_解除。早く外すとレンダリング中のフレームが「未処理」と誤認され二重発注される
      std::lock_guard<std::mutex> lock(qmtx_);
      pending_.erase(job);
    }
  }
}

} // namespace mu
