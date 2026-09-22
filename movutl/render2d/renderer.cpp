#include <movutl/asset/camera.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/group.hpp>
#include <movutl/asset/scene_change.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/render2d/composite_ops.hpp>
#include <movutl/render2d/renderer.hpp>
#include <set>
#include <string>

namespace mu {

bool CPURenderer::render_frame(Composition* comp, int frame, Ref<Image>& out, bool transparent_bg) {
  MOVUTL_ZONE_SCOPED_N("CPURenderer::render_frame");
  MU_ASSERT(comp != nullptr);

  if(!out) {
    out = cutil::make_ref<Image>(comp->size[0], comp->size[1]);
  } else if(out->width != comp->size[0] || out->height != comp->size[1]) {
    MOVUTL_ZONE_SCOPED_N("CPURenderer::resize");
    out->resize(Vec2d(comp->size[0], comp->size[1]));
  }

  {
    MOVUTL_ZONE_SCOPED_N("CPURenderer::fill_bg");
    if(transparent_bg) {
      // ネストされたCompositionはAfter Effectsのプリコンポジション同様、常に透明背景で合成する
      out->fill_rgba(Vec4b(0, 0, 0, 0));
    } else {
      uint32_t bg = (uint32_t)comp->bg_color;
      out->fill_rgba(Vec4b{(unsigned char)(bg & 0xFF), (unsigned char)((bg >> 8) & 0xFF), (unsigned char)((bg >> 16) & 0xFF), (unsigned char)((bg >> 24) & 0xFF)});
    }
  }

  // comp->mtxはget_all_entities()内で短時間lockするのみ。Entity個々のレンダリング中はe->mtxだけをlockする
  const auto layered = [&] {
    MOVUTL_ZONE_SCOPED_N("CPURenderer::get_layered_entities");
    return comp->get_layered_entities();
  }();
  // クリッピング対象の1つ上のレイヤー番号(そのアルファをmaskへ保持する)
  std::set<int> mask_needed;
  for(auto& [li, ce] : layered)
    if(ce->clipping_up_ && ce->visible(frame)) mask_needed.insert(li - 1);
  Image mask;
  int mask_layer = -1000;
  for(auto& [layer_i, e] : layered) {
    MOVUTL_ZONE_SCOPED_N("CPURenderer::entity");
    {
      const std::string zn = "layer" + std::to_string(layer_i) + ":" + std::string(e->name.c_str());
      MOVUTL_ZONE_NAME(zn.c_str(), zn.size());
    }
    std::unique_lock<std::mutex> lock_try(e->mtx, std::defer_lock);
    {
      MOVUTL_ZONE_SCOPED_N("CPURenderer::entity_lock_wait"); // 他ワーカーがEntityを描画中だと待たされる
      lock_try.lock();
    }
    if(!e->visible(frame)) continue;
    {
      MOVUTL_ZONE_SCOPED_N("CPURenderer::apply_animated_props");
      e->apply_animated_props(frame); // 中間点アニメーションをframe時点の値へ評価してメンバ変数に反映する
    }

    // このEntityへ効くグループ制御を上のレイヤーから順に畳み込み、親変換として描画中のスレッドに与える(入れ子は外側から合成)
    GroupXform parent;
    bool has_parent = false;
    for(auto& [gl, g] : layered) {
      if(gl >= layer_i) break;
      if(g->getType() != EntityType_Group || !g->visible(frame)) continue;
      auto* ge = static_cast<GroupEntt*>(g.get());
      if(!ge->affects(gl, layer_i)) continue;
      parent     = parent.compose(ge->local_xform());
      has_parent = true;
    }
    // 「カメラ制御」ONのEntityには、上のレイヤーで最も近いカメラの視点変換を最外側の親として掛ける
    if(e->camera_ctrl_) {
      const Camera3D* cam = nullptr;
      for(auto& [cl, c] : layered) {
        if(cl >= layer_i) break;
        if(c->getType() != EntityType_Camera || !c->visible(frame)) continue;
        auto* ce = static_cast<Camera3D*>(c.get());
        if(ce->affects(cl, layer_i)) cam = ce;
      }
      if(cam) {
        parent     = cam->view_xform().compose(parent);
        has_parent = true;
      }
    }
    GroupXformScope parent_scope(has_parent ? &parent : nullptr);

    // このEntityへ効くシーンチェンジ(直下=1つ上のレイヤーに置かれたもの)。同フレームに複数あれば先勝ち
    const SceneChangeEntt* sc = nullptr;
    for(auto& [sl, s] : layered) {
      if(sl >= layer_i) break;
      if(s->getType() != EntityType_SceneChange || !s->visible(frame)) continue;
      auto* se = static_cast<SceneChangeEntt*>(s.get());
      if(se->affects(sl, layer_i)) {
        sc = se;
        break;
      }
    }
    if(sc && e->getType() != EntityType_SceneChange) {
      // outgoing A = 同レイヤーでeより前に終わった直前のオブジェクト(終端フレームを保持して描く)。無ければ透明(=フェードイン等に縮退)
      // ponytail: Aは終端フレームで静止(動画の再生継続は無し)。継続再生は素材フレームをfend_超えで引く実装が要る
      Entity* a = nullptr;
      for(auto& [al, ae] : layered)
        if(al == layer_i && ae.get() != e.get() && ae->fend_ < e->fstart_ && (!a || ae->fend_ > a->fend_)) a = ae.get();
      composite_scene_change(comp, a, e.get(), sc, out.get(), frame);
    } else
      // AviUtl「上のオブジェクトでクリッピング」: 1つ上(layer_i-1)のオブジェクトの形(アルファ)で切り抜く。上に可視オブジェクトが無ければ通常描画
      if(e->clipping_up_ && mask_layer == layer_i - 1) {
        composite_clipping_up(comp, e.get(), mask, out.get(), frame);
      } else {
        MOVUTL_ZONE_SCOPED_N("CPURenderer::entity_render");
        e->render(comp, out.get(), frame);
      }
    if(mask_needed.count(layer_i)) {
      // 直後のレイヤーのクリッピング用に、このオブジェクト単体のアルファを保持する
      mask.resize(Vec2d(out->width, out->height));
      mask.fill_rgba(Vec4b(0, 0, 0, 0));
      e->render(comp, &mask, frame);
      mask_layer = layer_i;
    }
  }
  {
    MOVUTL_ZONE_SCOPED_N("CPURenderer::dirty");
    out->dirty();
  }
  return true;
}

} // namespace mu
