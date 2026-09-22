#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>

namespace mu {

class Composition;
class SceneChangeEntt;

// シーンチェンジ中のeをoutへ合成する。aは同レイヤーで直前に終了したオブジェクト(outgoing。無ければnullptr)
void composite_scene_change(Composition* comp, Entity* a, Entity* e, const SceneChangeEntt* sc, Image* out, int frame);

// AviUtl「上のオブジェクトでクリッピング」を合成する。mask: 1つ上のレイヤーのアルファを保持したImage
void composite_clipping_up(Composition* comp, Entity* e, const Image& mask, Image* out, int frame);

} // namespace mu
