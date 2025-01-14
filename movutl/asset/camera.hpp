#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/core/core.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/quotanion.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

enum class CamDistModel : int16_t {
  NONE,                   /**< Rectilinear images. No distortion compensation required. */
  MODIFIED_BROWN_CONRADY, /**< Equivalent to Brown-Conrady distortion, except that tangential distortion is applied to radially distorted points */
  INVERSE_BROWN_CONRADY,  /**< Equivalent to Brown-Conrady distortion, except undistorts image instead of distorting it */
  FTHETA,                 /**< F-Theta fish-eye distortion model */
  BROWN_CONRADY,          /**< Unmodified Brown-Conrady distortion model */
  KANNALA_BRANDT4,        /**< Four parameter Kannala Brandt distortion model */
  COUNT,                  /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
  R2ONLY                  // r^2だけのパラメータのDistortion model
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;


// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vecs and Matrices for use in OpenGL
class Camera3D : public Entity {
public:
  Vec3 pos;
  Quat rot;
  Vec3 up = Vec3(0, 0, 1);
  float fov = 60.0f;
  float aspect = 1.0f;

  // camera options
  float move_speed = 2.5f;
  float rotate_speed = 0.1f;
};

} // namespace mu
