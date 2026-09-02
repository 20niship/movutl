#include <algorithm>
#include <imgui.h>
#include <movutl/asset/composition.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/audio/fft.hpp>
#include <movutl/gui/fft_window.hpp>
#include <vector>

namespace mu {

void FFTWindow::Update() {
  ImGui::Begin("FFT");
  auto comp = Composition::GetActiveComp();
  if(!comp || !comp->audio_buf) {
    ImGui::Text("No audio");
    ImGui::End();
    return;
  }

  constexpr int kN   = 2048; // 2の冪
  int channels       = std::max(1, comp->audio_channels);
  int64_t cur_sample = comp->frame_to_sample(comp->frame);
  std::vector<int16_t> pcm((size_t)kN * channels, 0);
  comp->audio_buf->snapshot(cur_sample, kN, pcm.data());

  std::vector<int16_t> mono(kN);
  for(int i = 0; i < kN; i++) {
    int16_t l = pcm[(size_t)i * channels];
    int16_t r = channels > 1 ? pcm[(size_t)i * channels + 1] : l;
    mono[i]   = (int16_t)(((int32_t)l + r) / 2);
  }

  auto spectrum = compute_spectrum(mono.data(), kN); // kN/2点

  ImVec2 avail = ImGui::GetContentRegionAvail();
  if(avail.x >= 1 && avail.y >= 1) {
    auto dl       = ImGui::GetWindowDrawList();
    ImVec2 origin = ImGui::GetCursorScreenPos();
    int bars      = std::min((int)spectrum.size(), (int)avail.x);
    for(int x = 0; x < bars; x++) {
      int i   = x * (int)spectrum.size() / bars;
      float h = std::clamp(spectrum[i], 0.0f, 1.0f) * avail.y;
      dl->AddLine(ImVec2(origin.x + x, origin.y + avail.y), ImVec2(origin.x + x, origin.y + avail.y - h), IM_COL32(120, 200, 220, 255));
    }
    ImGui::Dummy(avail);
  }

  ImGui::End();
}

} // namespace mu
