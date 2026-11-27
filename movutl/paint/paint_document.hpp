#pragma once

#include <memory>
#include <movutl/core/enums.hpp>
#include <movutl/paint/paint_edit.hpp>
#include <movutl/paint/stroke.hpp>
#include <string>
#include <vector>

namespace mu {

// レイヤーの属性(画素以外)。set_layer_props()でまとめて変更する
struct LayerProps {
  std::string name{"レイヤー"};
  bool visible                             = true;
  float opacity                            = 1.0f;
  BlendType blend                          = Blend_Alpha;
  bool operator==(const LayerProps&) const = default;
};

struct PaintLayer : LayerProps {
  TiledImage img;
};

struct PaintRect {
  int x = 0, y = 0, w = 0, h = 0;
};

// 静止画のペイント文書(Compositionとは独立)。レイヤー列 + ストローク + 合成 + 保存
// 変更する操作はPaintEditを返す。undoしたい場合はapp/paint_commandsのpush_paint_edit()で共通の履歴へ積む
class PaintDocument {
public:
  PaintDocument(int w, int h) : w_(w), h_(h) {}
  PaintDocument(const PaintDocument&)            = delete; // PaintEditがアドレスを参照するのでコピー/ムーブ不可
  PaintDocument& operator=(const PaintDocument&) = delete;

  int width() const { return w_; }
  int height() const { return h_; }
  std::vector<PaintLayer> layers; // [0]が一番下。直接書き換えた場合はmark_changed()を呼ぶ

  int add_layer(const std::string& name); // 末尾(一番上)へ追加する。undoしない(新規作成・読込用)

  // undoできるレイヤー操作。適用してから取り消し用のEditを返す。範囲外・変化なしはnullptr。追加/削除/移動はストローク中もnullptr
  std::unique_ptr<PaintEdit> insert_layer(int index, const std::string& name); // index: 0〜layers.size()
  std::unique_ptr<PaintEdit> remove_layer(int index);                          // 最後の1枚は消せない
  std::unique_ptr<PaintEdit> move_layer(int from, int to);
  std::unique_ptr<PaintEdit> set_layer_props(int index, const LayerProps& props);

  // ストローク。begin〜endの間にadd_sample()で点を足す
  bool begin_stroke(int layer, const BrushSettings& s);
  void add_sample(const PenSample& p);
  void tick(float dt);                             // ストローク中に時間を進める(エアブラシ等。ペンが止まっていても呼ぶ)
  void set_clip(const Selection* s) { clip_ = s; } // 以降のストロークを選択範囲の内側に限る(nullで解除)
  std::unique_ptr<PaintEdit> end_stroke();         // 画素が変わらなければnullptrで、revisionも開始前に戻る
  void cancel_stroke();                            // 描画中のストロークを無かったことにする(revisionも開始前に戻る)
  bool stroking() const { return stroke_ != nullptr; }

  // 画素やレイヤーが変わるたびに増える。値は文書をまたいで重複しないので、前回と比べるだけで表示の更新を判定できる
  uint64_t revision() const { return revision_; }
  void mark_changed() {
    all_dirty_       = true;
    stroke_revision_ = 0; // ストローク以外の変更があるので、終了時に開始前の値へは戻せない
    bump_revision();
  }

  // 可視レイヤーを下から合成してoutへ書く(outはw x hにリサイズされる)。bgは下地色(既定は透明)
  void composite(PixelBuffer8& out, const Vec4b& bg = Vec4b()) const;
  // 表示用の差分合成。前回の呼び出し以降に変わった所だけoutを合成し直し、その矩形を返す。
  // ストローク以外の変更があった場合とoutの大きさが違う場合は全面を合成して全体の矩形1つを返す。呼び出し元は1か所だけにする
  std::vector<PaintRect> composite_dirty(PixelBuffer8& out, const Vec4b& bg = Vec4b());
  Vec4b pick(int x, int y) const; // 合成結果の1画素。範囲外は透明

  // .mup形式(独自バイナリ: マジック+JSONヘッダ+未圧縮タイル)。失敗時false
  bool save(const std::string& path) const;
  static std::unique_ptr<PaintDocument> load(const std::string& path);

  std::weak_ptr<PaintDocument*> handle() const { return self_; } // PaintEdit用の生存確認つき参照

private:
  void bump_revision() { revision_ = ++last_revision_; } // 画素の変更だけ(変わったタイルはTiledImageが覚える)
  void revert_revision();                                // 画素がストローク開始前と同じになった時に呼ぶ
  void composite_tile(PixelBuffer8& out, int tx, int ty, const Vec4b& bg) const;

  int w_, h_;
  bool all_dirty_                       = true;
  static inline uint64_t last_revision_ = 0;
  uint64_t revision_                    = ++last_revision_;
  uint64_t stroke_revision_             = 0; // ストローク開始前のrevision。0なら戻さない
  std::unique_ptr<Stroke> stroke_;
  const Selection* clip_                = nullptr;
  int stroke_layer_                     = -1;
  std::shared_ptr<PaintDocument*> self_ = std::make_shared<PaintDocument*>(this);
};

} // namespace mu
