#include <movutl/asset/entity.hpp>
#include <movutl/asset/group.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/render2d/renderer.hpp>
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
    GroupXformScope parent_scope(has_parent ? &parent : nullptr);

    if(e->clipping_up_) {
      // ponytail: 単一共有バッファ逐次合成のため未描画の上レイヤーは参照不可。既に合成済みの下側アルファをマスクに使う近似実装(真の上レイヤークリッピングには2パスレンダリングが必要)
      MOVUTL_ZONE_SCOPED_N("CPURenderer::clipping_up");
      Image scratch(out->width, out->height);
      scratch.fill_rgba(Vec4b(0, 0, 0, 0));
      e->render(comp, &scratch, frame);
      for(size_t i = 0; i < out->size(); i++) {
        Vec4b s    = scratch[i];
        Vec4b& d   = (*out)[i];
        float mask = d[3] / 255.0f; // 既存(下)のアルファをマスクにする
        float a    = (s[3] / 255.0f) * mask;
        if(a <= 0.0f) continue;
        for(int c = 0; c < 3; c++) d[c] = (unsigned char)(s[c] * a + d[c] * (1.0f - a));
        d[3] = (unsigned char)(a * 255.0f + d[3] * (1.0f - a));
      }
    } else {
      MOVUTL_ZONE_SCOPED_N("CPURenderer::entity_render");
      e->render(comp, out.get(), frame);
    }
  }
  {
    MOVUTL_ZONE_SCOPED_N("CPURenderer::dirty");
    out->dirty();
  }
  return true;
}

} // namespace mu
