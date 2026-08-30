#pragma once
#include <cutil/ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/core/assert.hpp>

namespace mu::test {

struct VisualTestScene {
  Ref<Composition> comp;
  Ref<ShapeEntt> rect;
  Ref<ShapeEntt> circle;
  Ref<TextEntt> text;
  Ref<Movie> movie;
};

// video_pathを背景に、赤い矩形(左上)/緑の円(中央やや下)/白いテキスト(左下)を重ねた検証用シーンを構築する
inline VisualTestScene make_visual_test_scene(const char* video_path, int nframes) {
  VisualTestScene s;
  s.movie = Movie::Create("bg_movie", video_path);
  MU_ASSERT(s.movie->get_input_plugin());
  s.movie->trk.fstart = 0;
  s.movie->trk.fend   = nframes - 1;

  const auto w = (float)s.movie->get_info().width;
  const auto h = (float)s.movie->get_info().height;

  s.rect             = ShapeEntt::Create("rect", ShapeType_Rect);
  s.rect->pos_       = Vec3(w * 0.05f, h * 0.05f, 0);
  s.rect->size_      = Vec2(w * 0.2f, h * 0.2f);
  s.rect->color_     = Vec4b(0, 0, 255, 255); // BGRA順: 赤
  s.rect->trk.fstart = 0;
  s.rect->trk.fend   = nframes - 1;

  s.circle             = ShapeEntt::Create("circ", ShapeType_Circle);
  s.circle->pos_       = Vec3(w * 0.5f, h * 0.4f, 0);
  s.circle->size_      = Vec2(w * 0.2f, h * 0.2f);
  s.circle->color_     = Vec4b(0, 255, 0, 255); // BGRA順: 緑
  s.circle->trk.fstart = 0;
  s.circle->trk.fend   = nframes - 1;

  s.text             = TextEntt::Create("HI");
  s.text->pos_       = Vec3(w * 0.05f, h * 0.75f, 0);
  s.text->color_     = Vec4b(255, 255, 255, 255);
  s.text->trk.fstart = 0;
  s.text->trk.fend   = nframes - 1;

  s.comp = cutil::make_ref<Composition>("VisualTestComp", (int)w, (int)h, (int)s.movie->get_info().framerate);
  s.comp->insert_entity(s.movie, -1);  // layer0(背景)
  s.comp->insert_entity(s.rect, -1);   // layer1
  s.comp->insert_entity(s.circle, -1); // layer2
  s.comp->insert_entity(s.text, -1);   // layer3(最前面)
  return s;
}

} // namespace mu::test
