#include <imgui.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/app/wnd_developper.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/time.hpp>

namespace mu {

void DeveloperWindow::Update() {
  ImGui::Begin("開発者ウィンドウ");

  auto& pool = detail::AppMain::Get()->render_pool;

  static uint64_t last_rendered = 0;
  static double last_sample_t   = mu_now_seconds();
  static float render_fps       = 0.0f;
  uint64_t rendered             = pool.total_rendered();
  double now                    = mu_now_seconds();
  if(now - last_sample_t >= 0.5) {
    render_fps    = (float)((rendered - last_rendered) / (now - last_sample_t));
    last_rendered = rendered;
    last_sample_t = now;
  }

  ImGui::Text("レンダー速度: %.1f frame/s", render_fps);
  ImGui::Text("累計レンダー済フレーム数: %llu", (unsigned long long)rendered);
  ImGui::Text("キュー待ち: %zu", pool.queue_size());

  auto comp = Composition::GetActiveComp();
  if(comp) {
    int cur_frame;
    {
      std::lock_guard<std::mutex> lock(comp->mtx);
      cur_frame = comp->frame;
    }
    ImGui::Text("main threadフレーム: %d", cur_frame);
    ImGui::Text("キャッシュ済フレーム数: %zu", comp->cache.size());
  } else {
    ImGui::Text("main threadフレーム: (Compositionなし)");
  }

  ImGui::Separator();
  ImGui::Text("ワーカースレッド (%zu)", pool.worker_count());
  auto statuses = pool.worker_status();
  for(size_t i = 0; i < statuses.size(); i++) {
    const auto& s = statuses[i];
    if(s.busy)
      ImGui::Text("  worker[%zu]: rendering frame %d", i, s.frame);
    else
      ImGui::TextDisabled("  worker[%zu]: idle", i);
  }

  ImGui::End();
}

} // namespace mu
