#include <doctest/doctest.h>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/core/audio_resample.hpp>

using namespace mu;

TEST_CASE("audio_resample-passthrough") {
  int16_t in[4] = {100, -100, 200, -200}; // 2ch x 2samples
  int16_t out[4] = {0, 0, 0, 0};
  audio_resample(in, 2, 2, out, 2, 2, 1.0);
  CHECK(out[0] == 100);
  CHECK(out[1] == -100);
  CHECK(out[2] == 200);
  CHECK(out[3] == -200);
}

TEST_CASE("audio_resample-mono-to-stereo") {
  int16_t in[2]  = {1000, 2000}; // mono, 2 samples
  int16_t out[4] = {0, 0, 0, 0};
  audio_resample(in, 2, 1, out, 2, 2, 1.0);
  CHECK(out[0] == 1000);
  CHECK(out[1] == 1000);
  CHECK(out[2] == 2000);
  CHECK(out[3] == 2000);
}

TEST_CASE("AudioRingBuffer-write-then-snapshot") {
  AudioRingBuffer rb(1000, 2, 1.0); // 1000Hz, stereo, 1秒分
  int16_t data[6] = {1, 2, 3, 4, 5, 6}; // 3サンプル分
  rb.write(0, data, 3);

  int16_t out[6] = {0};
  rb.snapshot(0, 3, out);
  for(int i = 0; i < 6; i++) CHECK(out[i] == data[i]);
}

TEST_CASE("AudioRingBuffer-unwritten-range-is-silence") {
  AudioRingBuffer rb(1000, 1, 1.0);
  int16_t out[4] = {9, 9, 9, 9};
  rb.snapshot(500, 4, out);
  for(int i = 0; i < 4; i++) CHECK(out[i] == 0);
}
