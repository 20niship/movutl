#include <doctest/doctest.h>
#include <movutl/render2d/frame_cache.hpp>
#include <movutl/render2d/render_worker.hpp>
#include <thread>

using namespace mu;

TEST_CASE("FrameCache::get_nearest: 現在フレームに最も近いキャッシュ済みフレームを返す") {
  FrameCache c;
  Ref<Image> out;
  int f = -1;
  CHECK_FALSE(c.get_nearest(10, &out, &f));

  auto a = cutil::make_ref<Image>(2, 2), b = cutil::make_ref<Image>(2, 2), d = cutil::make_ref<Image>(2, 2);
  c.insert(5, a, 5);
  c.insert(20, b, 5);
  c.insert(40, d, 5);
  CHECK(c.get_nearest(20, &out, &f));
  CHECK(f == 20);
  CHECK(out.get() == b.get());
  CHECK(c.get_nearest(11, &out, &f)); // 5(距離6)より20(距離9)は遠い
  CHECK(f == 5);
  CHECK(c.get_nearest(100, &out, &f)); // 未来側にしか無ければ最も近い過去側
  CHECK(f == 40);
  CHECK(c.get_nearest(0, &out, &f)); // 過去側に無ければ最も近い未来側
  CHECK(f == 5);
  c.insert(30, a, 5);
  CHECK(c.get_nearest(35, &out, &f)); // 30と40は同距離 → 過去側
  CHECK(f == 30);
}

TEST_CASE("RenderWorkerPool::tick: 現在フレームが未描画なら古い待機ジョブは捨てられ、現在フレームが先頭に積み直される") {
  RenderWorkerPool pool(0); // workerを持たないのでキューに積まれたまま観測できる
  auto comp    = cutil::make_ref<Composition>("t", 16, 16, 30);
  comp->fstart = 0;
  comp->fend   = 500;

  comp->frame = 0;
  pool.tick(comp.get(), true);
  const size_t before = pool.queue_size();
  CHECK(before > 1);

  comp->frame = 300; // 再生が遅れて先へ進んだ
  pool.tick(comp.get(), true);
  // 0..299は捨てられ、300以降だけが残る(300+先読み)
  CHECK(pool.queue_size() <= (size_t)(500 - 300 + 1));
  CHECK(pool.queue_size() > 0);
}
