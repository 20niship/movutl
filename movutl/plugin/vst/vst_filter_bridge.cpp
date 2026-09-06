#include <algorithm>
#include <cmath>
#include <memory>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/prop_types.hpp>
#include <movutl/plugin/plugin.hpp>
#include <movutl/plugin/vst/vst_filter_bridge.hpp>
#include <movutl/plugin/vst/vst_host.hpp>
#include <remidy/remidy.hpp>
#include <uapmd-plugin-hosting/uapmd-plugin-hosting.hpp>

namespace mu::detail {

namespace {

constexpr uint64_t kVstFilterGuidPrefix = 0x0757000000000000ULL;

// VST3プラグインごとのfn_proc状態。processAudio用のコンテキストをplugin_idごとに1つ保持し呼び出しのたびに使い回す
struct VstFilterStateFull : VstFilterState {
  remidy::MasterContext master;
  std::unique_ptr<remidy::AudioProcessContext> ctx;
  int channels        = 0;
  int capacity_frames = 0;
  bool started        = false;
};

bool vst_fn_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_ASSERT(fpip != nullptr);
  if(fpip->audiop == nullptr || fpip->audio_n <= 0) return false;
  std::string plugin_id = cutil::get_or<std::string>(p, "vst_plugin_id", "");
  if(plugin_id.empty()) return false;

  auto* slot = (void**)fp;
  if(*slot == nullptr) {
    auto* st        = new VstFilterStateFull();
    int sample_rate = fpip->compo ? fpip->compo->audio_sample_rate : 48000;
    st->instance_id = vst_host::create_instance(plugin_id, (uint32_t)sample_rate, (uint32_t)fpip->audio_n);
    if(st->instance_id < 0) {
      LOG_F(ERROR, "vst_fn_proc: プラグインのインスタンス化に失敗: %s", plugin_id.c_str());
      delete st;
      return false; // *slotはnullptrのまま。次回呼び出しでも再試行する
    }
    *slot = st;
  }
  auto* st = (VstFilterStateFull*)*slot;
  if(st->instance_id < 0) return false;
  auto* inst = vst_host::get_instance(st->instance_id);
  if(inst == nullptr) return false;

  int n  = fpip->audio_n;
  int ch = std::max(1, fpip->audio_ch);
  if(!st->ctx || st->capacity_frames < n || st->channels != ch) {
    st->ctx = std::make_unique<remidy::AudioProcessContext>(st->master, 4096);
    st->ctx->configureMainBus(ch, ch, n);
    st->channels        = ch;
    st->capacity_frames = n;
    st->started         = false;
  }
  st->master.sampleRate(fpip->compo ? fpip->compo->audio_sample_rate : 48000);
  st->ctx->frameCount(n);
  st->ctx->clearAudioInputs();

  for(int c = 0; c < ch; c++) {
    float* buf = st->ctx->getFloatInBuffer(0, c);
    if(buf == nullptr) continue;
    for(int i = 0; i < n; i++) buf[i] = fpip->audiop[i * ch + c] / 32768.0f;
  }

  if(!st->started) {
    inst->startProcessing();
    st->started = true;
  }
  inst->processAudio(*st->ctx);

  for(int c = 0; c < ch; c++) {
    float* buf = st->ctx->getFloatOutBuffer(0, c);
    if(buf == nullptr) continue;
    for(int i = 0; i < n; i++) {
      int32_t v               = (int32_t)std::lround(buf[i] * 32768.0f);
      fpip->audiop[i * ch + c] = (int16_t)std::clamp(v, -32768, 32767);
    }
  }
  return true;
}

bool vst_fn_exit(void* fp) {
  auto* slot = (void**)fp;
  if(*slot != nullptr) {
    auto* st = (VstFilterStateFull*)*slot;
    if(st->instance_id >= 0) vst_host::destroy_instance(st->instance_id);
    delete st;
    *slot = nullptr;
  }
  return true;
}

FilterPluginTable build_table(const vst_host::PluginInfo& info, size_t index) {
  FilterPluginTable t{};
  t.guid              = kVstFilterGuidPrefix | (index & 0x0000FFFFFFFFFFFFULL);
  t.flag              = FilterAudioOnly;
  t.name              = cutil::Str(info.name.c_str());
  t.info              = cutil::Str("VST3エフェクト");
  t.version           = 0;
  t.version_str       = "0";
  t.fn_cutstom_wnd    = nullptr;
  t.fn_update_value   = nullptr;
  t.fn_init           = nullptr; // defaultsは登録時に直接設定するため不要
  t.fn_exit           = vst_fn_exit;
  t.fn_proc           = vst_fn_proc;
  t.fn_update         = nullptr;
  t.func_is_saveframe = nullptr;
  t.fn_project_load   = nullptr;
  t.func_project_save = nullptr;

  // plugin_idはfn_proc側からはテーブルを識別できないため、プロパティ経由(非表示フィールド)で持ち回す
  t.props.fields.push_back(cutil::PropInfo::Field("vst_plugin_id", 0, cutil::prop_info_of<std::string>()));
  t.props.fields.back().flags = cutil::PropFlags::Hidden;
  t.defaults.set<std::string>("vst_plugin_id", info.id);
  return t;
}

} // namespace

uapmd_plugin_hosting::AudioPluginInstanceAPI* vst_filter_instance(void* instance_state) {
  if(instance_state == nullptr) return nullptr;
  return vst_host::get_instance(((VstFilterState*)instance_state)->instance_id);
}

bool is_vst_filter_guid(uint64_t guid) { return (guid & 0xFFFF000000000000ULL) == kVstFilterGuidPrefix; }

void register_vst_filters() {
  auto plugins  = vst_host::plugin_list();
  auto& filters = AppMain::Get()->filters;
  for(size_t i = 0; i < plugins.size(); i++) {
    LOG_F(1, "Registering VST3 filter: %s (%s)", plugins[i].name.c_str(), plugins[i].id.c_str());
    filters.push_back(build_table(plugins[i], i));
  }
}

} // namespace mu::detail
