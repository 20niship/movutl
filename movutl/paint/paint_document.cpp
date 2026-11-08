#include <fstream>
#include <movutl/paint/paint_document.hpp>
#include <movutl/raster/blend.hpp>
#include <nlohmann/json.hpp>

namespace mu {

int PaintDocument::add_layer(const std::string& name) {
  layers.emplace_back();
  layers.back().name = name;
  layers.back().img  = TiledImage(w_, h_);
  return (int)layers.size() - 1;
}

bool PaintDocument::begin_stroke(int layer, const BrushSettings& s) {
  if(layer < 0 || layer >= (int)layers.size() || stroke_) return false;
  stroke_layer_ = layer;
  layers[layer].img.begin_edit();
  stroke_ = std::make_unique<BrushStroke>(&layers[layer].img, s);
  return true;
}

void PaintDocument::add_sample(const PenSample& p) {
  if(stroke_) stroke_->add(p);
}

void PaintDocument::end_stroke() {
  if(!stroke_) return;
  stroke_.reset();
  auto patches = layers[stroke_layer_].img.end_edit();
  if(!patches.empty()) {
    undo_.push_back({stroke_layer_, std::move(patches)});
    redo_.clear();
  }
  stroke_layer_ = -1;
}

bool PaintDocument::undo() {
  if(undo_.empty() || stroke_) return false;
  auto step = std::move(undo_.back());
  undo_.pop_back();
  for(auto& p : step.patches) layers[step.layer].img.set_tile(p.tx, p.ty, p.before);
  redo_.push_back(std::move(step));
  return true;
}

bool PaintDocument::redo() {
  if(redo_.empty() || stroke_) return false;
  auto step = std::move(redo_.back());
  redo_.pop_back();
  for(auto& p : step.patches) layers[step.layer].img.set_tile(p.tx, p.ty, p.after);
  undo_.push_back(std::move(step));
  return true;
}

void PaintDocument::composite(PixelBuffer8& out, const Vec4b& bg) const {
  out.resize(w_, h_);
  out.fill(bg);
  for(const auto& L : layers) {
    if(!L.visible || L.opacity <= 0.0f) continue;
    with_blend(L.blend, [&](auto B) {
      for(const auto& [k, tile] : L.img.tiles()) {
        const int ox = TiledImage::key_tx(k) * TiledImage::kTile, oy = TiledImage::key_ty(k) * TiledImage::kTile;
        for(int ty = 0; ty < TiledImage::kTile && oy + ty < h_; ty++) {
          for(int tx = 0; tx < TiledImage::kTile && ox + tx < w_; tx++) {
            Vec4b s       = tile(tx, ty);
            const float a = s[3] / 255.0f * L.opacity;
            if(a <= 0.0f) continue;
            Vec4b& d = out(ox + tx, oy + ty);
            if constexpr(decltype(B)::value != Blend_Alpha) {
              // 下地(d)が不透明なほど合成式の結果を使い、透明なら元の色のまま
              const float da = d[3] / 255.0f;
              for(int c = 0; c < 3; c++) s[c] = (unsigned char)(s[c] + (blend_channel<decltype(B)::value>(d[c], s[c]) - s[c]) * da + 0.5f);
            }
            blend_over_straight(d, s, a);
          }
        }
      }
    });
  }
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
  std::ofstream f(path, std::ios::binary);
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
  return (bool)f;
}

std::unique_ptr<PaintDocument> PaintDocument::load(const std::string& path) {
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
}

} // namespace mu
