#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>

namespace mu {

class Movie : public Entity {
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
  ~Movie();

  Ref<Image> img_;
  std::string path_;
  static Ref<Movie> Create(const char* name, const char* path = nullptr);
};

} // namespace mu
