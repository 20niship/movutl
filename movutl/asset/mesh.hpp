#pragma once
#include <movutl/asset/image.hpp>
#include <movutl/core/rect.hpp>
#include <movutl/core/vector.hpp>

namespace mu {
class Render;
}

namespace mu {

using IndicesT = uint32_t;

#define MAX_BONE_INFLUENCE 3

class Mesh {
  friend class Render;

public:
  enum class MeshTopology {
    TRIANGLES,
    TRIANGLE_STRIP,
    LINES,
    LINE_STRIP,
    POINTS,
  };

  MeshTopology topo = MeshTopology::TRIANGLES;
  Vec<IndicesT> indices;

  struct Vertex {
    float pos[3];
    uint8_t col[3];
    float uv[2];
    float norm[3];
    Vertex() {
      pos[0] = pos[1] = pos[2] = 0.0;
      col[0] = col[1] = col[2] = 0.0;
      uv[0] = uv[1] = 0.0;
    }
    template <typename T, typename U> Vertex(const _Vec<T, 3>& _pos, const Vec3b& _col, const _Vec<U, 2>& _uv) {
      pos[0] = _pos[0];
      pos[1] = _pos[1];
      pos[2] = _pos[2];
      col[0] = _col[0];
      col[1] = _col[1];
      col[2] = _col[2];
      uv[0] = _uv[0];
      uv[1] = _uv[1];
    }

    template <typename T> Vertex(const _Vec<T, 3>& _pos, const Vec3b& _col) {
      pos[0] = _pos[0];
      pos[1] = _pos[1];
      pos[2] = _pos[2];
      col[0] = _col[0];
      col[1] = _col[1];
      col[2] = _col[2];
      uv[0] = uv[1] = 0;
    }
  };
  Vec<Vertex> vertices;

  Vec3 center_of_geom() const;
  Rect3D get_bbox() const;
  virtual void move_geom(const Vec3&);  /// すべてのMeshについて 第1引数だけ移動する
  virtual void scale_geom(const Vec3&); /// すべてのMeshについて 第1引数scale倍する
  virtual size_t get_vertices_size() = 0;
  virtual void rot_geom(const Vec3&); /// すべてのMeshについて 第1引数だけ回転する
  Vec3 get_center_of_geom() const;

  size_t get_indices_size() { return indices.size(); }
  Mesh() = default;
  ~Mesh() = default;
};

} // namespace mu
