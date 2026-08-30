#include <algorithm>
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
  auto key = std::make_pair(comp, frame);
  if(pending_.count(key)) return;
  pending_.insert(key);
  if(urgent)
    queue_.push_front(Job{comp, frame});
  else
    queue_.push_back(Job{comp, frame});
  cv_.notify_one();
}

void RenderWorkerPool::tick(Composition* comp, bool playing, int prefetch_ahead) {
  if(!comp) return;
  int cur;
  {
    std::lock_guard<std::mutex> lock(comp->mtx);
    cur = comp->frame;
  }
  if(!comp->cache.is_cached(cur)) request(comp, cur, true);
  // 停止中も待機中のワーカーがcurrent frame周辺を先読みし続けるようにする(再生中は先読み範囲を広く取る)
  int ahead = playing ? prefetch_ahead : std::min(prefetch_ahead, 10);
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
    int f      = worker_frame_[i].load();
    out[i]     = {f, f >= 0};
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
      pending_.erase(std::make_pair(job.comp, job.frame));
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
  }
}

} // namespace mu
