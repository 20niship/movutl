#pragma once
#include <movutl/asset/entity.hpp>

namespace mu {

class AudioEntt final : public Entity {
private:
  bool load_failed_ = false; // ロード失敗時の警告スパム防止フラグ

public:
  AudioEntt() = default;
  AudioEntt(const char* path);
  ~AudioEntt() = default;

  int start_frame_ = 0;      // MPROPERTY(name="開始フレーム")
  float speed      = 100.0f; // MPROPERTY(name="再生速度", min=0.0, max=1000.0, step=5.0)
  float volume_    = 100.0f; // MPROPERTY(name="音量", min=0.0, max=200.0)
  bool loop_       = false;  // MPROPERTY(name="ループ再生")
  bool mute_       = false;  // MPROPERTY(name="ミュート")
  std::string path_;         // MPROPERTY(name="ファイル", type="path")

  static Ref<AudioEntt> Create(const char* name, const char* path = nullptr);
  bool load_file(const char* path);
  virtual EntityType getType() const override { return EntityType_Audio; }
  virtual bool render(Composition* cmp, Image* target, int frame) override;
  virtual void reload_asset() override { load_file(path_.c_str()); }

  // 絶対サンプル位置[start_sample, start_sample+n)のPCM16(interleaved)をoutへ加算合成する
  bool fetch_audio(Composition* cmp, int64_t start_sample, int n, int16_t* out);

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
