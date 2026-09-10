// レンダー/音声ワーカーがComposition*を握ったまま破棄されるとdangling pointerでクラッシュする問題(Project::New/Load時)の回帰テスト
#include <atomic>
#include <doctest/doctest.h>
#include <movutl/asset/composition.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/render2d/render_worker.hpp>
#include <thread>

using namespace mu;

TEST_CASE("RenderWorkerPool::flush: flush完了後はワーカーがComposition*を保持していないため直後の破棄でも落ちない") {
  RenderWorkerPool pool(2);
  std::atomic<bool> stop_ticking{false};

  for(int iter = 0; iter < 30; iter++) {
    auto comp  = cutil::make_ref<Composition>("t", 64, 64, 30);
    comp->guid = 1000 + iter;
    stop_ticking.store(false);

    std::thread ticker([&] {
      while(!stop_ticking.load()) pool.tick(comp.get(), true);
    });

    std::this_thread::sleep_for(std::chrono::microseconds(200));
    stop_ticking.store(true);
    ticker.join();

    // Project::New/Loadがcompos_.clear()する直前に呼ぶのと同じ手順: flush()で実行中ジョブの完了を待ってから破棄する
    pool.flush();
    comp.reset(); // flushが効いていなければここでワーカーがdangling pointerを触りクラッシュする
  }

  auto status = pool.worker_status();
  for(auto& s : status) CHECK_FALSE(s.busy);
}

TEST_CASE("AudioMixWorker::pause: pause完了後はワーカーがComposition*を保持していないため直後の破棄でも落ちない") {
  AudioMixWorker worker;
  std::atomic<bool> stop_ticking{false};

  for(int iter = 0; iter < 30; iter++) {
    auto comp  = cutil::make_ref<Composition>("t", 64, 64, 30);
    comp->guid = 2000 + iter;
    stop_ticking.store(false);

    std::thread ticker([&] {
      while(!stop_ticking.load()) worker.tick(comp.get(), true);
    });

    std::this_thread::sleep_for(std::chrono::microseconds(200));
    stop_ticking.store(true);
    ticker.join();

    worker.pause();
    comp.reset(); // pauseが効いていなければここでワーカーがdangling pointerを触りクラッシュする
  }

  CHECK(true);
}
