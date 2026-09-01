#pragma once
#include <algorithm>
#include <cutil/ref.hpp>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/core/assert.hpp>

namespace mu::test {

constexpr int kSceneWidth  = 1280;
constexpr int kSceneHeight = 720;

struct VisualTestScene {
  Ref<Composition> comp;
  Ref<ShapeEntt> rect;
  Ref<ShapeEntt> circle;
  Ref<TextEntt> text;
  Ref<Movie> movie_a;
  Ref<Movie> movie_b;
  Ref<AudioEntt> audio;
};

// assets/movies配下の2本の動画を全画面に拡大して重ね、再生開始フレームをずらした上へ赤い矩形(左上)/緑の円(中央やや下)/白いテキスト(左下)を重ねた検証用シーンを構築する
inline Ref<Movie> make_fullscreen_movie(const char* name, const char* path, int fstart, int nframes) {
  auto mov = Movie::Create(name, path);
  MU_ASSERT(mov->get_input_plugin());
  mov->fstart          = fstart;
  mov->fend            = fstart + nframes - 1;
  const float native_w = (float)mov->get_info().width;
  const float native_h = (float)mov->get_info().height;
  const float cover    = std::max((float)kSceneWidth / native_w, (float)kSceneHeight / native_h) * 100.0f;
  mov->scale           = Vec2(cover, cover);
  return mov;
}

inline VisualTestScene make_visual_test_scene(int nframes) {
  VisualTestScene s;
  s.movie_a = make_fullscreen_movie("bg_movie_a", "../assets/movies/big_buck_bunny_360_10s.mp4", 0, nframes);
  s.movie_b = make_fullscreen_movie("bg_movie_b", "../assets/movies/sample-5s.mp4", 5, nframes); // 開始フレームをずらす

  constexpr float w = (float)kSceneWidth;
  constexpr float h = (float)kSceneHeight;

  s.rect         = ShapeEntt::Create("rect", ShapeType_Rect);
  s.rect->pos_   = Vec3(w * 0.05f, h * 0.05f, 0);
  s.rect->size_  = Vec2(w * 0.2f, h * 0.2f);
  s.rect->color_ = Vec4b(0, 0, 255, 255); // BGRA順: 赤
  s.rect->fstart = 0;
  s.rect->fend   = nframes - 1;

  s.circle         = ShapeEntt::Create("circ", ShapeType_Circle);
  s.circle->pos_   = Vec3(w * 0.5f, h * 0.4f, 0);
  s.circle->size_  = Vec2(w * 0.2f, h * 0.2f);
  s.circle->color_ = Vec4b(0, 255, 0, 255); // BGRA順: 緑
  s.circle->fstart = 0;
  s.circle->fend   = nframes - 1;

  s.text         = TextEntt::Create("HI");
  s.text->pos_   = Vec3(w * 0.05f, h * 0.75f, 0);
  s.text->color_ = Vec4b(255, 255, 255, 255);
  s.text->fstart = 0;
  s.text->fend   = nframes - 1;

  s.audio         = AudioEntt::Create("audio", "../assets/audio/file_example_WAV_1MG.wav");
  s.audio->fstart = 0;
  s.audio->fend   = nframes - 1;

  s.comp = cutil::make_ref<Composition>("VisualTestComp", kSceneWidth, kSceneHeight, (int)s.movie_a->get_info().framerate);
  s.comp->insert_entity(s.movie_a, -1); // layer0(背景)
  s.comp->insert_entity(s.movie_b, -1); // layer1(背景の上に重畳)
  s.comp->insert_entity(s.rect, -1);    // layer2
  s.comp->insert_entity(s.circle, -1);  // layer3
  s.comp->insert_entity(s.text, -1);    // layer4(最前面)
  s.comp->insert_entity(s.audio, -1);   // layer5(音声)
  return s;
}

} // namespace mu::test
