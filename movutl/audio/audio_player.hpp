#pragma once

namespace mu {

class Composition;

// miniaudioを用いた再生デバイスの薄いラッパー。Composition::audio_bufから読み出してデバイスへ渡すだけ
class AudioPlayer {
public:
  AudioPlayer();
  ~AudioPlayer();

  AudioPlayer(const AudioPlayer&)            = delete;
  AudioPlayer& operator=(const AudioPlayer&) = delete;

  // 再生対象のCompositionを切り替える(サンプルレート/チャンネル数が変わるためデバイスを再初期化する)
  void set_composition(Composition* comp);
  void start();
  void stop();
  bool is_running() const { return running_; }

  // プレビュー再生専用のマスター音量(線形ゲイン、1.0=0dB)。書き出し・波形・メーターには影響しない
  static void set_master_gain(float gain);
  static float master_gain();

private:
  void* device_      = nullptr; // ma_device*(実体はaudio_player.cppで確保、ヘッダをmovutl全体へ波及させないためvoid*で保持)
  Composition* comp_ = nullptr;
  bool running_      = false;
};

} // namespace mu
