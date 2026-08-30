#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <movutl/asset/composition.hpp>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace mu {

// Compositionのフレームをバックグラウンドで常時レンダリングし、Composition::cacheへ書き込むワーカースレッドプール
class RenderWorkerPool {
public:
  explicit RenderWorkerPool(size_t n_workers = default_worker_count());
  ~RenderWorkerPool();

  RenderWorkerPool(const RenderWorkerPool&)            = delete;
  RenderWorkerPool& operator=(const RenderWorkerPool&) = delete;

  // 現在フレームを優先(urgent)でキューイングし、Config::cache_framesの空き分だけ先読みをキューイングする
  void tick(Composition* comp, bool playing);

  void request(Composition* comp, int frame, bool urgent);

  void stop();

  static size_t default_worker_count();

  // 開発者ウィンドウ表示用のワーカー状態(1ワーカーにつき現在レンダリング中のフレーム、-1ならidle)
  struct WorkerStatus {
    int frame = -1;
    bool busy = false;
  };
  size_t worker_count() const { return workers_.size(); }
  std::vector<WorkerStatus> worker_status() const;
  size_t queue_size() const;
  uint64_t total_rendered() const { return total_rendered_.load(); }

private:
  struct Job {
    Composition* comp;
    int frame;
    bool operator<(const Job& o) const { return comp != o.comp ? comp < o.comp : frame < o.frame; }
  };

  void worker_loop(size_t worker_idx);

  std::vector<std::thread> workers_;
  std::deque<Job> queue_;
  std::set<Job> pending_; // キュー投入済み〜レンダリング完了(cache挿入)までを表す。二重発注防止用
  mutable std::mutex qmtx_;
  std::condition_variable cv_;
  std::atomic<bool> stop_{false};

  std::unique_ptr<std::atomic<int>[]> worker_frame_;
  std::atomic<uint64_t> total_rendered_{0};
};

} // namespace mu
