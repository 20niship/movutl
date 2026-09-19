#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/audio/audio_player.hpp>
#include <movutl/gui/audio_meter.hpp>
#include <vector>

namespace mu {

namespace {
constexpr int kSnapshotN  = 1024;
constexpr float kMinDb    = -48.0f;
constexpr float kMaxDb    = 6.0f; // フェーダーの上限
constexpr float kHoldSec  = 1.0f;
constexpr float kDecayDbS = 12.0f;

float to_db(float lin) { return 20.0f * std::log10(std::max(lin, 1e-6f)); }
float from_db(float db) { return db <= kMinDb ? 0.0f : std::pow(10.0f, db / 20.0f); }
float meter_t(float db) { return std::clamp((db - kMinDb) / -kMinDb, 0.0f, 1.0f); }

ImU32 meter_color(float db) {
  if(db > -3.0f) return IM_COL32(235, 70, 70, 255);
  if(db > -12.0f) return IM_COL32(235, 205, 70, 255);
  return IM_COL32(70, 205, 115, 255);
}

ImU32 dim(ImU32 c, float k) {
  ImVec4 v = ImGui::ColorConvertU32ToFloat4(c);
  return ImGui::ColorConvertFloat4ToU32(ImVec4(v.x * k, v.y * k, v.z * k, 1.0f));
}

// ピークホールドとクリップ表示はUI状態としてファイルスコープに持つ(メーターは1つだけ)
struct MeterState {
  float peak_db[2]   = {kMinDb, kMinDb};
  double peak_time[2] = {0, 0};
  bool clip[2]        = {false, false};
  float pre_mute_gain = 1.0f;
  bool gain_synced    = false;
} g_meter;

void draw_one_meter(ImDrawList* dl, float x0, float y0, float w, float h, int ch, float level_db, float peak_db, bool active) {
  for(int py = 0; py < (int)h; py++) {
    const float t  = 1.0f - (float)py / h;
    const float db = kMinDb + t * -kMinDb;
    ImU32 col      = meter_color(db);
    if(!active || t > meter_t(level_db)) col = dim(col, active ? 0.22f : 0.12f);
    dl->AddRectFilled(ImVec2(x0, y0 + py), ImVec2(x0 + w, y0 + py + 1), col);
  }
  if(active && peak_db > kMinDb) {
    const float y = y0 + h * (1.0f - meter_t(peak_db));
    dl->AddLine(ImVec2(x0, y), ImVec2(x0 + w, y), IM_COL32(255, 255, 255, 230), 1.5f);
  }
  (void)ch;
}
} // namespace

bool audio_current_levels(Composition* comp, AudioLevels& lv) {
  if(!comp || !comp->audio_buf) return false;
  const int channels = std::max(1, comp->audio_channels);
  std::vector<int16_t> pcm((size_t)kSnapshotN * channels, 0);
  comp->audio_buf->snapshot(comp->frame_to_sample(comp->frame), kSnapshotN, pcm.data());
  double sum[2] = {0, 0};
  int peak[2]   = {0, 0};
  for(int i = 0; i < kSnapshotN; i++) {
    for(int c = 0; c < 2; c++) {
      const int v = pcm[(size_t)i * channels + (channels > 1 ? c : 0)];
      sum[c] += (double)v * v;
      peak[c] = std::max(peak[c], std::abs(v));
    }
  }
  for(int c = 0; c < 2; c++) {
    lv.rms[c]  = (float)(std::sqrt(sum[c] / kSnapshotN) / 32768.0);
    lv.peak[c] = peak[c] / 32768.0f;
  }
  return true;
}

void draw_audio_wave(ImDrawList* dl, const ImVec2& min, const ImVec2& max, Composition* comp) {
  dl->AddRectFilled(min, max, IM_COL32(20, 20, 20, 255), 3.0f);
  const float mid = (min.y + max.y) / 2.0f;
  const int w     = (int)(max.x - min.x);
  if(!comp || !comp->audio_buf || w < 2) {
    dl->AddLine(ImVec2(min.x, mid), ImVec2(max.x, mid), IM_COL32(120, 120, 120, 120));
    return;
  }
  const int channels = std::max(1, comp->audio_channels);
  std::vector<int16_t> pcm((size_t)kSnapshotN * channels, 0);
  comp->audio_buf->snapshot(comp->frame_to_sample(comp->frame), kSnapshotN, pcm.data());
  const float half = (max.y - min.y) / 2.0f - 1.0f;
  for(int x = 0; x < w; x++) {
    const int i0 = x * kSnapshotN / w, i1 = std::max(i0 + 1, (x + 1) * kSnapshotN / w);
    int amp = 0;
    for(int i = i0; i < i1 && i < kSnapshotN; i++) amp = std::max(amp, std::abs((int)pcm[(size_t)i * channels]));
    const float a = amp / 32768.0f * half;
    dl->AddLine(ImVec2(min.x + x, mid - a), ImVec2(min.x + x, mid + a + 1), IM_COL32(120, 220, 160, 220));
  }
}

TimelineRightStrip::TimelineRightStrip(Composition* comp) {
  ImGuiWindow* win = ImGui::GetCurrentWindow();
  win_             = win;
  saved_max_x_     = win->InnerClipRect.Max.x;
  const ImVec2 cur = ImGui::GetCursorScreenPos();
  const float x1   = saved_max_x_;
  const float x0   = x1 - kWidth;
  const float y0   = cur.y;
  const float y1   = win->InnerClipRect.Max.y;
  win->InnerClipRect.Max.x = x0;
  if(y1 - y0 < 60.0f) return;

  if(!g_meter.gain_synced) {
    AudioPlayer::set_master_gain(Config::Get()->master_volume);
    g_meter.gain_synced = true;
  }

  ImDrawList* dl = win->DrawList;
  dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(28, 28, 28, 255));
  dl->AddLine(ImVec2(x0, y0), ImVec2(x0, y1), IM_COL32(70, 70, 70, 255));

  AudioLevels lv;
  const bool has_audio = audio_current_levels(comp, lv);
  const float gain     = AudioPlayer::master_gain();
  const double now     = ImGui::GetTime();

  const float lamp_h = 8.0f, foot_h = 22.0f;
  const float my0 = y0 + lamp_h + 6.0f, my1 = y1 - foot_h - 4.0f;
  const float mh  = my1 - my0;
  const float lx = x0 + 22.0f, mw = 9.0f;
  const float fx = lx + mw * 2 + 9.0f, fw = 16.0f;

  for(int c = 0; c < 2; c++) {
    const float db = has_audio ? to_db(lv.rms[c] * gain) : kMinDb;
    const float pk = has_audio ? to_db(lv.peak[c] * gain) : kMinDb;
    if(pk >= g_meter.peak_db[c]) {
      g_meter.peak_db[c]   = pk;
      g_meter.peak_time[c] = now;
    } else if(now - g_meter.peak_time[c] > kHoldSec) {
      g_meter.peak_db[c] = std::max(kMinDb, g_meter.peak_db[c] - kDecayDbS * ImGui::GetIO().DeltaTime);
    }
    if(has_audio && lv.peak[c] * gain >= 0.999f) g_meter.clip[c] = true;

    const float bx = lx + c * (mw + 1.0f);
    draw_one_meter(dl, bx, my0, mw, mh, c, db, g_meter.peak_db[c], has_audio);
    const ImVec2 lp0(bx, y0 + 4.0f), lp1(bx + mw, y0 + 4.0f + lamp_h);
    dl->AddRectFilled(lp0, lp1, g_meter.clip[c] ? IM_COL32(235, 60, 60, 255) : IM_COL32(70, 30, 30, 255), 2.0f);
    ImGui::SetCursorScreenPos(lp0);
    ImGui::PushID(c);
    if(ImGui::InvisibleButton("##clip_lamp", ImVec2(mw, lamp_h))) g_meter.clip[c] = false;
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("クリップ表示をリセット");
    ImGui::PopID();
  }

  for(float db : {0.0f, -6.0f, -12.0f, -24.0f, -48.0f}) {
    const float y = my0 + mh * (1.0f - meter_t(db));
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%d", (int)db);
    const ImVec2 ts = ImGui::CalcTextSize(buf);
    dl->AddLine(ImVec2(lx - 3, y), ImVec2(lx, y), IM_COL32(150, 150, 150, 200));
    dl->AddText(ImVec2(lx - 5 - ts.x, std::clamp(y - ts.y / 2, my0 - 2, my1 - ts.y + 2)), IM_COL32(150, 150, 150, 220), buf);
  }

  {
    const float cx = fx + fw / 2;
    dl->AddRectFilled(ImVec2(cx - 2, my0), ImVec2(cx + 2, my1), IM_COL32(15, 15, 15, 255), 2.0f);
    dl->AddRect(ImVec2(cx - 2, my0), ImVec2(cx + 2, my1), IM_COL32(80, 80, 80, 255), 2.0f);
    const float y0db = my0 + mh * (1.0f - (0.0f - kMinDb) / (kMaxDb - kMinDb));
    dl->AddLine(ImVec2(fx, y0db), ImVec2(fx + fw, y0db), IM_COL32(150, 150, 150, 200));

    ImGui::SetCursorScreenPos(ImVec2(fx, my0));
    ImGui::InvisibleButton("##master_fader", ImVec2(fw, mh));
    const bool active = ImGui::IsItemActive();
    float cur_db      = gain <= 0.0f ? kMinDb : std::clamp(to_db(gain), kMinDb, kMaxDb);
    if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
      cur_db = 0.0f;
      AudioPlayer::set_master_gain(1.0f);
      Config::Get()->master_volume = 1.0f;
      Config::Save();
    } else if(active) {
      const float t = 1.0f - std::clamp((ImGui::GetMousePos().y - my0) / mh, 0.0f, 1.0f);
      cur_db        = kMinDb + t * (kMaxDb - kMinDb);
      if(std::fabs(cur_db) < 1.0f) cur_db = 0.0f;
      const float g = from_db(cur_db);
      AudioPlayer::set_master_gain(g);
      Config::Get()->master_volume = g;
    }
    if(ImGui::IsItemDeactivatedAfterEdit()) Config::Save();
    const float ky = my0 + mh * (1.0f - (cur_db - kMinDb) / (kMaxDb - kMinDb));
    const bool hov = ImGui::IsItemHovered() || active;
    dl->AddRectFilled(ImVec2(fx, ky - 4), ImVec2(fx + fw, ky + 4), hov ? IM_COL32(235, 235, 235, 255) : IM_COL32(190, 190, 190, 255), 2.0f);
    dl->AddLine(ImVec2(fx + 2, ky), ImVec2(fx + fw - 2, ky), IM_COL32(40, 40, 40, 255));
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("マスター音量(プレビュー再生のみ)\nダブルクリックで0dB");
  }

  {
    const float fy = y1 - foot_h;
    const bool muted = gain <= 0.0f;
    ImGui::SetCursorScreenPos(ImVec2(x0 + 4, fy));
    if(ImGui::InvisibleButton("##master_mute", ImVec2(18, foot_h))) {
      if(muted) {
        AudioPlayer::set_master_gain(g_meter.pre_mute_gain);
        Config::Get()->master_volume = g_meter.pre_mute_gain;
      } else {
        g_meter.pre_mute_gain = gain;
        AudioPlayer::set_master_gain(0.0f);
        Config::Get()->master_volume = 0.0f;
      }
      Config::Save();
    }
    dl->AddText(ImVec2(x0 + 5, fy + 3), muted ? IM_COL32(235, 90, 90, 255) : IM_COL32(200, 200, 200, 255), muted ? ICON_FA_VOLUME_XMARK : ICON_FA_VOLUME_HIGH);
    if(ImGui::IsItemHovered()) ImGui::SetTooltip(muted ? "ミュート解除" : "ミュート");
    char buf[16];
    if(muted || gain <= 0.0f)
      std::snprintf(buf, sizeof(buf), "-inf");
    else
      std::snprintf(buf, sizeof(buf), "%+.1f", to_db(gain));
    const ImVec2 ts = ImGui::CalcTextSize(buf);
    dl->AddText(ImVec2(x1 - ts.x - 3, fy + 3), IM_COL32(190, 190, 190, 255), buf);
  }

  ImGui::SetCursorScreenPos(cur);
}

TimelineRightStrip::~TimelineRightStrip() { static_cast<ImGuiWindow*>(win_)->InnerClipRect.Max.x = saved_max_x_; }

} // namespace mu
