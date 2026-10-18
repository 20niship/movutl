#pragma once
#include <condition_variable>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>
#include <mutex>
#include <set>

namespace mu {

class Movie final : public Entity {
private:
  bool load_failed_ = false; ///< ロード失敗時の警告スパム防止フラグ

  // fn_read_videoをtlocal昇順で実行させるゲート(無いとRenderWorkerPoolの先読みで到達順が乱れ無駄な逆シークが起きる)
  std::mutex decode_order_mtx_;
  std::condition_variable decode_order_cv_;
  std::multiset<int> pending_decode_;

  // fstart_/fend_/speed/start_frame_/loop_からtlocal(素材内フレーム)を計算する。範囲外かつループ無しなら-1
  int compute_tlocal(int frame) const;

  void wait_decode_order(int tlocal);
  void release_decode_order(int tlocal);
  friend struct DecodeOrderGuard; // movie.cppでwait/release_decode_orderをRAII化するため

public:
  Movie() = default;
  Movie(const char* path);
  ~Movie() = default;

  Ref<Image> img_;
  int start_frame_ = 0;     // MPROPERTY(name="開始フレーム")
  float speed      = 100.0; // MPROPERTY(name="再生速度", min=0.0, max=10000.0, step=5.0)
  bool loop_       = false; // MPROPERTY(name="ループ再生")
  bool with_alpha_ = false; // MPROPERTY(name="透明度を読み込む")
  std::string path_;        // MPROPERTY(name="ファイル", type="path")

  static Ref<Movie> Create(const char* name, const char* path = nullptr);
  bool load_file(const char* path);
  virtual EntityType getType() const override { return EntityType_Movie; }
  virtual bool render(Composition* cmp, Image* target, int frame) override;
  virtual bool source_size(Vec2& size, Vec2& origin_offset) const override;
  virtual void reload_asset() override { load_file(path_.c_str()); }

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
