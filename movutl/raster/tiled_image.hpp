#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <movutl/core/assert.hpp>
#include <movutl/raster/pixel_buffer.hpp>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mu {

// 1タイル分の変更前後(Undo/Redo・自動保存の差分単位)。before/afterが無いのは「タイルが存在しなかった」状態
struct TilePatch {
  int tx = 0, ty = 0;
  std::optional<PixelBuffer8> before;
  std::optional<PixelBuffer8> after;
};

// 8bit RGBA(ストレートアルファ)のスパースなタイル画像。描かれていないタイルはメモリを持たず、読み出すと完全透明
class TiledImage {
public:
  static constexpr int kTile = 64;

  TiledImage() = default;
  TiledImage(int w, int h) : w_(w), h_(h) {}

  int width() const { return w_; }
  int height() const { return h_; }
  size_t tile_count() const { return tiles_.size(); }
  const std::unordered_map<uint64_t, PixelBuffer8>& tiles() const { return tiles_; }

  static uint64_t key(int tx, int ty) { return ((uint64_t)(uint32_t)tx << 32) | (uint32_t)ty; }
  static int key_tx(uint64_t k) { return (int)(uint32_t)(k >> 32); }
  static int key_ty(uint64_t k) { return (int)(uint32_t)k; }

  bool in_range(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }

  Vec4b get(int x, int y) const {
    if(!in_range(x, y)) return Vec4b();
    auto it = tiles_.find(key(x / kTile, y / kTile));
    return it == tiles_.end() ? Vec4b() : it->second(x % kTile, y % kTile);
  }
  // タイルが無ければnullptr
  const PixelBuffer8* find(int tx, int ty) const {
    auto it = tiles_.find(key(tx, ty));
    return it == tiles_.end() ? nullptr : &it->second;
  }

  // 前回の呼び出し以降に書き換えた(かもしれない)タイルのキーをoutへ移す。表示の差分更新に使う
  void take_dirty(std::unordered_set<uint64_t>& out) {
    out.merge(dirty_);
    dirty_.clear();
    last_dirty_ = kNoKey;
  }

  // 書き込み用。タイルを確保し、begin_edit()中なら初回接触時にタイルの変更前を記録する
  Vec4b& at(int x, int y) {
    MU_ASSERT(in_range(x, y));
    const int tx = x / kTile, ty = y / kTile;
    const uint64_t k = key(tx, ty);
    if(k != last_dirty_) dirty_.insert(last_dirty_ = k); // 同じタイルが続く間はハッシュを引かない
    auto it = tiles_.find(k);
    if(recording_ && !patch_index_.count(k)) {
      TilePatch p;
      p.tx = tx;
      p.ty = ty;
      if(it != tiles_.end()) p.before = it->second;
      patch_index_[k] = patches_.size();
      patches_.push_back(std::move(p));
    }
    if(it == tiles_.end()) it = tiles_.emplace(k, PixelBuffer8(kTile, kTile)).first;
    return it->second(x % kTile, y % kTile);
  }

  // begin_edit()〜end_edit()の間に触ったタイルの差分を集める(ネスト不可)
  void begin_edit() {
    recording_ = true;
    patches_.clear();
    patch_index_.clear();
  }
  // 触っただけで画素が変わらなかったタイルは差分に含めない(そのために確保したタイルは解放する)
  std::vector<TilePatch> end_edit() {
    std::vector<TilePatch> changed;
    for(auto& p : patches_) {
      const uint64_t k = key(p.tx, p.ty);
      auto it          = tiles_.find(k);
      if(it != tiles_.end()) {
        if(same_pixels(it->second, p.before)) {
          if(!p.before) tiles_.erase(it);
          dirty_.insert(k); // 途中の状態が表示済みかもしれない
          continue;
        }
        p.after = it->second;
      }
      changed.push_back(std::move(p));
    }
    recording_ = false;
    patches_.clear();
    patch_index_.clear();
    return changed;
  }

  // タイルをbufで置き換える(nulloptなら削除)。Undo/Redo・読み込みで使う
  void set_tile(int tx, int ty, const std::optional<PixelBuffer8>& buf) {
    dirty_.insert(key(tx, ty));
    if(buf)
      tiles_[key(tx, ty)] = *buf;
    else
      tiles_.erase(key(tx, ty));
  }

private:
  // bが無いのは完全透明と同じ
  static bool same_pixels(const PixelBuffer8& a, const std::optional<PixelBuffer8>& b) {
    if(b) return std::memcmp(a.data(), b->data(), a.size_in_bytes()) == 0;
    const auto* bytes = reinterpret_cast<const uint8_t*>(a.data());
    return std::all_of(bytes, bytes + a.size_in_bytes(), [](uint8_t v) { return v == 0; });
  }

  static constexpr uint64_t kNoKey = ~0ull;
  int w_ = 0, h_ = 0;
  std::unordered_map<uint64_t, PixelBuffer8> tiles_;
  std::unordered_set<uint64_t> dirty_;
  uint64_t last_dirty_ = kNoKey;
  bool recording_      = false;
  std::vector<TilePatch> patches_;
  std::unordered_map<uint64_t, size_t> patch_index_;
};

} // namespace mu
