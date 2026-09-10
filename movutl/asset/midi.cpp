#include <algorithm>
#include <cmath>
#include <cstring>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/midi.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/vst/vst_host.hpp>
#include <remidy/remidy.hpp>
#include <uapmd-plugin-hosting/uapmd-plugin-hosting.hpp>

namespace mu {

// fn_procの呼び出しごとに使い回すremidy側の処理コンテキスト(vst_filter_bridge.cppのVstFilterStateFullと同型)
struct MidiEntt::ProcState {
  remidy::MasterContext master;
  std::unique_ptr<remidy::AudioProcessContext> ctx;
  int channels        = 0;
  int capacity_frames = 0;
  bool started        = false;
};

MidiEntt::MidiEntt() = default;

MidiEntt::~MidiEntt() {
  if(instrument_instance_id_ >= 0) vst_host::destroy_instance(instrument_instance_id_);
}

Ref<MidiEntt> MidiEntt::Create(const char* name) {
  auto m  = cutil::make_ref<MidiEntt>();
  m->name = name;
  Project::Get()->entities.push_back(m);
  m->guid_ = Project::Get()->entities.size();
  return m;
}

bool MidiEntt::assign_instrument(const std::string& pluginId) {
  if(instrument_instance_id_ >= 0) {
    vst_host::destroy_instance(instrument_instance_id_);
    instrument_instance_id_ = -1;
    proc_.reset(); // サンプルレート/バス構成が新しいインスタンスと食い違わないよう作り直す
  }
  auto* cmp               = Composition::GetActiveComp();
  uint32_t sr             = cmp ? (uint32_t)cmp->audio_sample_rate : 48000u;
  instrument_instance_id_ = vst_host::create_instance(pluginId, sr, 1024);
  if(instrument_instance_id_ < 0) {
    LOG_F(ERROR, "MidiEntt::assign_instrument: プラグインのインスタンス化に失敗: %s", pluginId.c_str());
    return false;
  }
  instrument_plugin_id_ = pluginId;
  return true;
}

bool MidiEntt::render(Composition* cmp, Image* target, int frame) {
  MU_UNUSED(cmp);
  MU_UNUSED(target);
  MU_UNUSED(frame);
  return true; // MidiEnttは画像を描画しない(AudioEnttと同様)
}

bool MidiEntt::fetch_audio(Composition* cmp, int64_t start_sample, int n, int16_t* out) {
  MU_ASSERT(cmp != nullptr);
  MU_ASSERT(out != nullptr);
  if(!active_ || instrument_instance_id_ < 0) return false;
  auto* inst = vst_host::get_instance(instrument_instance_id_);
  if(inst == nullptr) return false;

  int64_t track_start = cmp->frame_to_sample(fstart_);
  int64_t track_end   = cmp->frame_to_sample(fend_);
  if(track_end <= track_start) return false;
  if(start_sample + n <= track_start || start_sample >= track_end) return false; // このチャンクはトラック範囲外

  int ch = std::max(1, cmp->audio_channels);
  if(!proc_) proc_ = std::make_unique<ProcState>();
  if(!proc_->ctx || proc_->capacity_frames < n || proc_->channels != ch) {
    proc_->ctx = std::make_unique<remidy::AudioProcessContext>(proc_->master, 4096);
    proc_->ctx->configureMainBus(0, ch, n); // 音源プラグインなので入力バスは持たない
    proc_->channels        = ch;
    proc_->capacity_frames = n;
    proc_->started         = false;
  }
  proc_->master.sampleRate(cmp->audio_sample_rate);
  proc_->ctx->frameCount(n);
  proc_->ctx->clearAudioOutputs();

  auto& ev = proc_->ctx->eventIn();
  ev.position(0);
  auto push_ump = [&](uint32_t word) {
    size_t pos = ev.position();
    if(pos + sizeof(uint32_t) > ev.maxMessagesInBytes()) return;
    std::memcpy(static_cast<uint8_t*>(ev.getMessages()) + pos, &word, sizeof(uint32_t));
    ev.position(pos + sizeof(uint32_t));
  };
  // MIDI 1.0 Channel Voice UMP(32bit, group/channel = 0固定): [0x2<<28 | group<<24 | (status<<4|ch)<<16 | data1<<8 | data2]
  constexpr uint8_t kGroup = 0, kChannel = 0, kNoteOn = 0x9, kNoteOff = 0x8;
  auto make_midi1 = [](uint8_t status, uint8_t data1, uint8_t data2) -> uint32_t { return (uint32_t(0x2) << 28) | (uint32_t(kGroup) << 24) | (uint32_t((status << 4) | kChannel) << 16) | (uint32_t(data1) << 8) | uint32_t(data2); };
  for(const auto& note : notes_) {
    int64_t abs_start = track_start + note.start_sample;
    int64_t abs_end   = abs_start + std::max<int64_t>(0, note.dur_samples);
    if(abs_start >= start_sample && abs_start < start_sample + n) push_ump(make_midi1(kNoteOn, note.pitch & 0x7F, note.velocity & 0x7F));
    if(abs_end >= start_sample && abs_end < start_sample + n) push_ump(make_midi1(kNoteOff, note.pitch & 0x7F, 0));
  }

  if(!proc_->started) {
    inst->startProcessing();
    proc_->started = true;
  }
  inst->processAudio(*proc_->ctx);

  for(int c = 0; c < ch; c++) {
    float* buf = proc_->ctx->getFloatOutBuffer(0, (uint32_t)c);
    if(buf == nullptr) continue;
    for(int i = 0; i < n; i++) {
      int32_t v       = (int32_t)out[i * ch + c] + (int32_t)std::lround(buf[i] * 32768.0f);
      out[i * ch + c] = (int16_t)std::clamp(v, -32768, 32767);
    }
  }
  return true;
}

} // namespace mu
