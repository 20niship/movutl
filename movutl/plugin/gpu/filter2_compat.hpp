#pragma once
// AviUtl2
// filter2.h(ext/aviutl2_sdk_mirror/include/aviutl2_sdk/filter2.h)と形状互換のGPU描画/シェーダ実行API。filter2.hはID3D11*/LPCWSTR/BYTE等Windows/D3D11型に依存し直接includeできないため値だけを移植する。詳細規約はdocs/plan_gpu_renderer.mdのPhase7節、GLSL規約はmovutl/vulkan/gpu_compute.hpp参照。.cso(DXBC)と.spv(ランタイムがGLSL文字列しか受け付けないため)は非対応
#include <cstdint>
#include <memory>
#include <movutl/asset/image.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace mu {
class GpuImage;
}

namespace mu::detail {

// --- filter2.hの値をそのままコピー(filter2.hはincludeしない) ---

struct VERTEX_COLOR {
  float x, y, z;
  float r, g, b, a;
};
struct VERTEX_COLOR_NORM {
  float x, y, z;
  float r, g, b, a;
  float vx, vy, vz;
};
struct VERTEX_TEXTURE {
  float x, y, z;
  float u, v;
  float a;
};
struct VERTEX_TEXTURE_NORM {
  float x, y, z;
  float u, v;
  float a;
  float vx, vy, vz;
};

enum class VERTEX_TYPE : int {
  TRIANGLE_COLOR        = 1,
  TRIANGLE_COLOR_NORM   = 2,
  TRIANGLE_TEXTURE      = 3,
  TRIANGLE_TEXTURE_NORM = 4,
  QUAD_COLOR            = 5,
  QUAD_COLOR_NORM       = 6,
  QUAD_TEXTURE          = 7,
  QUAD_TEXTURE_NORM     = 8,
};

enum class BLEND_MODE : int {
  NONE       = 0,
  ADD        = 1,
  SUB        = 2,
  MUL        = 3,
  SCREEN     = 4,
  OVERLAY    = 5,
  LIGHT      = 6,
  DARK       = 7,
  BRIGHTNESS = 8,
  CHROMA     = 9,
  SHADOW     = 10,
  LIGHT_DARK = 11,
  DIFF       = 12,
};

// filter2.hのID3D11BlendState/ID3D11SamplerStateの簡易な代替(値のみ。get_blend_state/get_sampler_state相当は提供しない)
enum class BlendStateMode { COPY, MASK, DRAW, ADD };
enum class SamplerMode { CLIP, CLAMP, LOOP, MIRROR, DOT };

// wchar_t(LPCWSTR)の各要素を1コードポイントとみなしてUTF-8化する(Windows由来のUTF-16サロゲートペアは非対応、ASCII中心のリソース名/パス用途で十分)
std::string wstr_to_utf8(const wchar_t* s);

// filter2.hのリソースのライフタイムに合わせ、resource:xxxx/tempbufferはこのインスタンス限り、cache:xxxx/image:xxxx/randomはプロセス内で共有する
class Filter2Context {
public:
  // object_image: フィルタ対象の画像("object"にバインドされる)。framebuffer: 任意、無ければ"framebuffer"要求はfalse
  explicit Filter2Context(Image& object_image, Image* framebuffer = nullptr);
  ~Filter2Context();

  // "object"の内容をobject_imageへ書き戻す(呼び出し完了後に呼ぶこと)
  void sync_object_to_image();

  // resourceがdst(既定"object")と同じ場合、恒等変換(x=y=0,rot=0,scale=1)以外は読み書きの競合で結果が不定になる(ダブルバッファ未実装の既知の制限)
  bool draw_image(const wchar_t* resource, float x, float y, float z, float rx, float ry, float rz, float sx, float sy, float sz, float alpha);
  bool draw_image_to_resource(const wchar_t* dst_resource, const wchar_t* src_resource, float x, float y, float z, float rx, float ry, float rz, float sx, float sy, float sz, float alpha);

  bool draw_poly(VERTEX_TYPE vertex_type, const void* vertex_list, int vertex_num, const wchar_t* resource);
  bool draw_poly_to_resource(const wchar_t* dst_resource, VERTEX_TYPE vertex_type, const void* vertex_list, int vertex_num, const wchar_t* src_resource);

  void set_blend_mode(BLEND_MODE blend) { blend_mode_ = blend; }

  // シェーダは生GLSL文字列(data,data_size)またはファイル(.frag/.comp/.glslのみ、.cso/.spvはfalse+ログ)
  bool exec_pixelshader_data(const char* glsl_src, int src_size, const wchar_t* target, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size);
  bool exec_pixelshader_file(const wchar_t* shader_file, const wchar_t* target, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size);
  bool exec_computeshader_data(const char* glsl_src, int src_size, const wchar_t** target_list, int target_num, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size, int count_x, int count_y, int count_z);
  bool exec_computeshader_file(const wchar_t* shader_file, const wchar_t** target_list, int target_num, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size, int count_x, int count_y, int count_z);

  // get_image_data/set_image_data相当("object"のみ対応。他リソースはPhase7の範囲外)
  bool get_image_data(Vec4b* buffer);
  bool set_image_data(const Vec4b* buffer, int width, int height);

private:
  GpuImage* resolve(const std::wstring& name, bool create_if_missing);

  std::unordered_map<std::string, std::unique_ptr<GpuImage>> local_; // "resource:xxxx" / "tempbuffer"
  Image& object_image_;
  Image* framebuffer_image_;
  std::unique_ptr<GpuImage> object_gpu_;
  std::unique_ptr<GpuImage> framebuffer_gpu_;
  BLEND_MODE blend_mode_ = BLEND_MODE::NONE;
};

} // namespace mu::detail
