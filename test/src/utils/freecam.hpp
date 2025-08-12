#pragma once


#include "kmath/kmath.hpp"
#include "thirdparty/raylib/raylib.h"


struct FreeCam {
  Camera3D rl_camera;
  kmath::Vec3f position;
  kmath::Quatf direction;

  FreeCam()
    : rl_camera({
      .position = Vector3(),
      .target = Vector3(0.0, 0.0, -1.0),
      .up = Vector3(0.0, 1.0, 0.0),
      .fovy = 70,
      .projection = CAMERA_PERSPECTIVE,
    }),
    position(kmath::Vec3f::ZERO),
    direction(kmath::Quatf::IDENTITY)
  {}

  void update(float p_delta);
  void begin_mode_3d() const;
};
