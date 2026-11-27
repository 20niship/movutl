#include <algorithm>
#include <filesystem>
#include <fstream>
#include <movutl/paint/paint_document.hpp>
#include <movutl/paint/stroke_registry.hpp>
#include <movutl/raster/blend.hpp>
#include <nlohmann/json.hpp>

namespace mu {

int PaintDocument::add_layer(const std::string& name) {
  cancel_stroke(); // layersの再確保でストロークの書き込み先が無効になるのを防ぐ
  layers.emplace_back();
  layers.back().name = name;
  layers.back().img  = TiledImage(w_, h_);
  mark_changed();
  return (int)layers.size() - 1;
}

namespace {
// 呼ぶたびに「indexのレイヤーを抜いて預かる」と「預かったレイヤーを戻す」を交互に行う
std::function<void(PaintDocument&)> toggle_layer(int index, std::optional<PaintLayer> held) {
  return [index, held = std::move(held)](PaintDocument& d) mutable {
    if(held) {
      if(index > (int)d.layers.size()) return;
      d.layers.insert(d.layers.begin() + index, std::move(*held));
      held.reset();
    } else {
      if(index >= (int)d.layers.size()) return;
      held = std::move(d.layers[index]);
      d.layers.erase(d.layers.begin() + index);
    }
  };
}
std::unique_ptr<PaintEdit> apply_edit(PaintDocument& d, const char* name, std::function<void(PaintDocument&)> toggle) {
  toggle(d);
  d.mark_changed();
  return std::make_unique<LayerEdit>(d, name, std::move(toggle));
}
} // namespace

std::unique_ptr<PaintEdit> PaintDocument::insert_layer(int index, const std::string& name) {
  if(stroke_ || index < 0 || index > (int)layers.size()) return nullptr;
  PaintLayer L;
  L.name = name;
  L.img  = TiledImage(w_, h_);
  return apply_edit(*this, "レイヤーを追加", toggle_layer(index, std::move(L)));
}

std::unique_ptr<PaintEdit> PaintDocument::remove_layer(int index) {
  if(stroke_ || index < 0 || index >= (int)layers.size() || layers.size() <= 1) return nullptr;
  return apply_edit(*this, "レイヤーを削除", toggle_layer(index, std::nullopt));
}

std::unique_ptr<PaintEdit> PaintDocument::move_layer(int from, int to) {
  const int n = (int)layers.size();
  if(stroke_ || from < 0 || from >= n || to < 0 || to >= n || from == to) return nullptr;
  // 呼ぶたびにfrom→to, to→fromを交互に行う
  auto toggle = [from, to](PaintDocument& d) mutable {
    if(std::max(from, to) >= (int)d.layers.size()) return;
    auto b = d.layers.begin();
    if(from < to)
      std::rotate(b + from, b + from + 1, b + to + 1);
    else
      std::rotate(b + to, b + from, b + from + 1);
    std::swap(from, to);
  };
  return apply_edit(*this, "レイヤーを移動", std::move(toggle));
}

std::unique_ptr<PaintEdit> PaintDocument::set_layer_props(int index, const LayerProps& props) {
  // 属性はストロークの書き込み先(タイル)を動かさないので描画中も変えられる(名前の確定がキャンバスのクリックより後になる)
  if(index < 0 || index >= (int)layers.size() || props == static_cast<const LayerProps&>(layers[index])) return nullptr;
  auto toggle = [index, held = props](PaintDocument& d) mutable {
    if(index < (int)d.layers.size()) std::swap(held, static_cast<LayerProps&>(d.layers[index]));
  };
  return apply_edit(*this, "レイヤーの設定を変更", std::move(toggle));
}

bool PaintDocument::begin_stroke(int layer, const BrushSettings& s) {
  if(layer < 0 || layer >= (int)layers.size() || stroke_) return false;
  auto st = StrokeRegistry::Get().create(s.engine, &layers[layer].img, s);
  if(!st) return false;
  st->set_clip(clip_);
  stroke_layer_ = layer;
  layers[layer].img.begin_edit();
  stroke_          = std::move(st);
  stroke_revision_ = revision_;
  return true;
}

void PaintDocument::add_sample(const PenSample& p) {
  if(!stroke_) return;
  stroke_->add(p);
  bump_revision();
}

void PaintDocument::tick(float dt) {
  if(!stroke_) return;
  stroke_->tick(dt);
  // ponytail: tickで画素が変わるかは描画方式次第なので毎回変更扱い。変わったタイルが無ければ再合成は起きない
  bump_revision();
}

std::unique_ptr<PaintEdit> PaintDocument::end_stroke() {
  if(!stroke_) return nullptr;
  stroke_->end();
  stroke_.reset();
  const int layer = std::exchange(stroke_layer_, -1);
  auto patches    = layers[layer].img.end_edit();
  if(patches.empty()) {
    revert_revision();
    return nullptr;
  }
  bump_revision();
  return std::make_unique<StrokeEdit>(*this, layer, std::move(patches));
}

void PaintDocument::cancel_stroke() {
  if(!stroke_) return;
  stroke_.reset(); // end()は呼ばない(水彩等が確定の書き込みをしないように)
  auto& img = layers[std::exchange(stroke_layer_, -1)].img;
  for(const auto& p : img.end_edit()) img.set_tile(p.tx, p.ty, p.before);
  revert_revision();
}

void PaintDocument::revert_revision() {
  if(stroke_revision_)
    revision_ = stroke_revision_;
  else
    bump_revision();
}

namespace {
// レイヤー1画素を下地dへ重ねる
template <BlendType B> void blend_layer_pixel(Vec4b& d, Vec4b s, float opacity) {
  const float a = s[3] / 255.0f * opacity;
  if(a <= 0.0f) return;
  if constexpr(B != Blend_Alpha) {
    // 下地(d)が不透明なほど合成式の結果を使い、透明なら元の色のまま
    const float da = d[3] / 255.0f;
    for(int c = 0; c < 3; c++) s[c] = (unsigned char)(s[c] + (blend_channel<B>(d[c], s[c]) - s[c]) * da + 0.5f);
  }
  blend_over_straight(d, s, a);
}
} // namespace

void PaintDocument::composite_tile(PixelBuffer8& out, int tx, int ty, const Vec4b& bg) const {
  constexpr int T = TiledImage::kTile;
  const int ox = tx * T, oy = ty * T, w = std::min(T, w_ - ox), h = std::min(T, h_ - oy);
  for(int y = 0; y < h; y++) std::fill_n(&out(ox, oy + y), w, bg);
  for(const auto& L : layers) {
    if(!L.visible || L.opacity <= 0.0f) continue;
    const PixelBuffer8* tile = L.img.find(tx, ty);
    if(!tile) continue;
    with_blend(L.blend, [&](auto B) {
      for(int y = 0; y < h; y++)
        for(int x = 0; x < w; x++) blend_layer_pixel<decltype(B)::value>(out(ox + x, oy + y), (*tile)(x, y), L.opacity);
    });
  }
}

void PaintDocument::composite(PixelBuffer8& out, const Vec4b& bg) const {
  if(out.width() != w_ || out.height() != h_) out.resize(w_, h_);
  const int ntx = (w_ + TiledImage::kTile - 1) / TiledImage::kTile, nty = (h_ + TiledImage::kTile - 1) / TiledImage::kTile;
#pragma omp parallel for schedule(dynamic, 8)
  for(int i = 0; i < ntx * nty; i++) composite_tile(out, i % ntx, i / ntx, bg);
}

std::vector<PaintRect> PaintDocument::composite_dirty(PixelBuffer8& out, const Vec4b& bg) {
  std::unordered_set<uint64_t> keys;
  for(auto& L : layers) L.img.take_dirty(keys);
  if(std::exchange(all_dirty_, false) || out.width() != w_ || out.height() != h_) {
    composite(out, bg);
    return {{0, 0, w_, h_}};
  }
  std::vector<PaintRect> rects;
  for(const uint64_t k : keys) {
    const int x = TiledImage::key_tx(k) * TiledImage::kTile, y = TiledImage::key_ty(k) * TiledImage::kTile;
    if(x >= 0 && y >= 0 && x < w_ && y < h_) rects.push_back({x, y, std::min(TiledImage::kTile, w_ - x), std::min(TiledImage::kTile, h_ - y)});
  }
#pragma omp parallel for if(rects.size() > 32)
  for(int i = 0; i < (int)rects.size(); i++) composite_tile(out, rects[i].x / TiledImage::kTile, rects[i].y / TiledImage::kTile, bg);
  return rects;
}

Vec4b PaintDocument::pick(int x, int y) const {
  Vec4b d;
  for(const auto& L : layers) {
    if(!L.visible || L.opacity <= 0.0f) continue;
    with_blend(L.blend, [&](auto B) { blend_layer_pixel<decltype(B)::value>(d, L.img.get(x, y), L.opacity); });
  }
  return d;
}

namespace {
constexpr char kMagic[4]      = {'M', 'U', 'P', '1'};
constexpr uint32_t kMaxHeader = 1u << 24;
constexpr int kMaxSize        = 65536;
} // namespace

bool PaintDocument::save(const std::string& path) const {
  nlohmann::json j;
  j["version"] = 1;
  j["width"]   = w_;
  j["height"]  = h_;
  j["layers"]  = nlohmann::json::array();
  for(const auto& L : layers) j["layers"].push_back({{"name", L.name}, {"visible", L.visible}, {"opacity", L.opacity}, {"blend", (int)L.blend}, {"tiles", L.img.tile_count()}});
  const std::string head = j.dump();
  const std::string tmp  = path + ".tmp"; // 書き込みに失敗しても既存のファイルを壊さない
  std::ofstream f(tmp, std::ios::binary);
  if(!f) return false;
  const uint32_t hl = (uint32_t)head.size();
  f.write(kMagic, 4);
  f.write((const char*)&hl, 4);
  f.write(head.data(), head.size());
  for(const auto& L : layers) {
    for(const auto& [k, tile] : L.img.tiles()) {
      const int32_t t[2] = {TiledImage::key_tx(k), TiledImage::key_ty(k)};
      f.write((const char*)t, sizeof(t));
      f.write((const char*)tile.data(), tile.size_in_bytes());
    }
  }
  f.close();
  std::error_code ec;
  if(f) std::filesystem::rename(tmp, path, ec);
  if(f && !ec) return true;
  std::filesystem::remove(tmp, ec);
  return false;
}

std::unique_ptr<PaintDocument> PaintDocument::load(const std::string& path) try {
  std::ifstream f(path, std::ios::binary);
  if(!f) return nullptr;
  char magic[4];
  uint32_t hl = 0;
  if(!f.read(magic, 4) || std::string(magic, 4) != std::string(kMagic, 4) || !f.read((char*)&hl, 4) || hl == 0 || hl > kMaxHeader) return nullptr;
  std::string head(hl, '\0');
  if(!f.read(head.data(), hl)) return nullptr;
  auto j = nlohmann::json::parse(head, nullptr, false);
  if(j.is_discarded() || !j.contains("width") || !j.contains("height") || !j.contains("layers") || !j["layers"].is_array()) return nullptr;
  const int w = j["width"].get<int>(), h = j["height"].get<int>();
  if(w <= 0 || h <= 0 || w > kMaxSize || h > kMaxSize) return nullptr;
  auto doc = std::make_unique<PaintDocument>(w, h);
  for(const auto& lj : j["layers"]) {
    const int li   = doc->add_layer(lj.value("name", std::string("レイヤー")));
    auto& L        = doc->layers[li];
    L.visible      = lj.value("visible", true);
    L.opacity      = lj.value("opacity", 1.0f);
    L.blend        = (BlendType)lj.value("blend", 0);
    const size_t n = lj.value("tiles", (size_t)0);
    const int ntx = (w + TiledImage::kTile - 1) / TiledImage::kTile, nty = (h + TiledImage::kTile - 1) / TiledImage::kTile;
    for(size_t i = 0; i < n; i++) {
      int32_t t[2];
      PixelBuffer8 tile(TiledImage::kTile, TiledImage::kTile);
      if(!f.read((char*)t, sizeof(t)) || t[0] < 0 || t[1] < 0 || t[0] >= ntx || t[1] >= nty) return nullptr;
      if(!f.read((char*)tile.data(), tile.size_in_bytes())) return nullptr;
      L.img.set_tile(t[0], t[1], tile);
    }
  }
  return doc;
} catch(const nlohmann::json::exception&) {
  return nullptr; // ヘッダの値の型が違う
}

} // namespace mu
