#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
//
#include <movutl/asset/entity.hpp>

namespace mu {

// ピアノロールで編集する1つのMIDIノート。start_sample/dur_samplesはトラック開始位置(trk.fstart)からの相対サンプル数
struct MidiNote {
  uint8_t pitch        = 60;  // MIDIノート番号(0-127, 60=C4)
  uint8_t velocity     = 100; // ベロシティ(0-127)
  int64_t start_sample = 0;
  int64_t dur_samples  = 0;
};

// VST3音源(uapmd_plugin_hosting::AudioPluginInstanceAPI)でノート列を再生するEntity。AudioEnttと対称の構造
class MidiEntt final : public Entity {
private:
  struct ProcState; // remidy::MasterContext/AudioProcessContextの保持(.cppでのみ定義、ヘッダをremidy非依存に保つ)
  std::unique_ptr<ProcState> proc_;
  int instrument_instance_id_ = -1; // vst_host::create_instanceが返すインスタンスID(実行時状態、非保存)
  // pygenのMPROPERTY/Lua自動bindはstd::vector<構造体>のフィールド型に未対応のためprivateにする(既知の制約、notes_は現状プロジェクトファイルへ未保存)
  std::vector<MidiNote> notes_;

public:
  MidiEntt();
  ~MidiEntt();

  std::string instrument_plugin_id_; // MPROPERTY(name="音源プラグインID")

  static Ref<MidiEntt> Create(const char* name);

  // vst_host::create_instance()で音源プラグインのインスタンスを生成し直す。失敗時はfalseを返す
  bool assign_instrument(const std::string& pluginId);
  int instrument_instance_id() const { return instrument_instance_id_; }

  // pygenのLua自動bindがconst/非const overloadを区別できないため非constの単一overloadのみ公開する
  std::vector<MidiNote>& notes() { return notes_; }

  virtual EntityType getType() const override { return EntityType_Midi; }
  virtual bool render(Composition* cmp, Image* target, int frame) override;

  // 絶対サンプル位置[start_sample, start_sample+n)に重なるnotesをUMP Note On/Offとして音源プラグインへ送り、processAudio()の出力(float)をint16化してoutへ加算合成する
  virtual bool fetch_audio(Composition* cmp, int64_t start_sample, int n, int16_t* out) override;

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
