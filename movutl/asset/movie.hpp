#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>

namespace mu {

class Movie final : public Entity {
public:
  enum Format {
    FormatRGB = 0,
    FormatRGBA = 2,
    FormatGRAYSCALE = 1,
    FormatBGR = 3,
  };

public:
  Movie() = default;
  Movie(const char* path);
  ~Movie() = default;

  Ref<Image> img_;
  int start_frame_ = 0;     // MUPROPERTY(name="開始フレーム")
  float speed = 100.0f;     // MUPROPERTY(name="再生速度")
  uint8_t alpha_ = 255;     // MUPROPERTY(name="透明度")
  bool loop_ = false;       // MUPROPERTY(name="ループ再生")
  bool with_alpha_ = false; // MUPROPERTY(name="透明度を読み込む")
  std::string path_;        // MUPROPERTY(name="ファイル", type="path")

  static Ref<Movie> Create(const char* name, const char* path = nullptr);
  bool load_file(const char* path);
  virtual EntityType getType() const override { return EntityType_Movie; }
  virtual bool render(Composition* cmp) override;

  virtual PropsInfo getPropsInfo() const override;    // MUFUNC_AUTOGEN
  virtual Props getProps() const override;            // MUFUNC_AUTOGEN
  virtual void setProps(const Props& props) override; // MUFUNC_AUTOGEN
};

} // namespace mu
