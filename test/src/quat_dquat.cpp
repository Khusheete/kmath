#include "kmath/kmath.hpp"
#include "tests.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thirdparty/raylib/raylib.h>
#include <array>


struct TestData {
  Camera3D camera;
  kmath::Quatf camera_rotation;
  kmath::DQuatf triangle_start;
  kmath::DQuatf triangle_end;
  kmath::Vec3f triangle[3];
};


float ping_pong(const double t) {
  return std::abs(std::fmod(t + 1.0, 2.0) - 1.0);
}


void *quat_dquat_init() {
  TestData *data = new TestData();
  data->camera = {
    .position   = (Vector3){0.0f, 0.0f, 5.0f},
    .target     = (Vector3){0.0f, 0.0f, 0.0f},
    .up         = (Vector3){0.0f, 1.0f, 0.0f},
    .fovy       = 45.0f,
    .projection = CAMERA_PERSPECTIVE,
  };
  data->camera_rotation = kmath::Quatf::from_axis_angle(kmath::Vec3f::Y, 0.5 * std::numbers::pi);
  data->triangle_start = kmath::DQuatf::from_axis_angle_translation(kmath::Vec3f::Y, 0.0, 2.0f * kmath::Vec3f::X);
  data->triangle_end = kmath::DQuatf::from_axis_angle_translation(kmath::Vec3f::Y, std::numbers::pi, -2.0f * kmath::Vec3f::X);
  data->triangle[0] = kmath::Vec3f(0.0 ,  std::sqrt(3.0) / 4.0, 0.0);
  data->triangle[1] = kmath::Vec3f(-0.5, -std::sqrt(3.0) / 4.0, 0.0);
  data->triangle[2] = kmath::Vec3f(0.5 , -std::sqrt(3.0) / 4.0, 0.0);
  std::cout << "INIT" << std::endl;
  return data;
}


void quat_dquat_run(void *data) {
  TestData *d = (TestData*)data;

  float time = GetTime();
  
  kmath::Quatf rot = kmath::slerp<float>(kmath::Quatf::IDENTITY, d->camera_rotation, 0.2 * std::numbers::pi * time);
  kmath::Vec3f pos = (kmath::Vec3f)rot.unit_conjugate(kmath::Quatf(0.0, 0.0, 5.0, 0.0));
  d->camera.position = reinterpret_cast<Vector3&>(pos);
  
  kmath::DQuatf transform = kmath::kenlerp<float>(d->triangle_start, d->triangle_end, ping_pong(time), 0.7);
  std::array<Vector3, 3> transformed_triangle = {};

  for (int i = 0; i < 3; i++) {
    kmath::Vec3f vertex = transform.unit_conjugate(kmath::DQuatf::from_point(d->triangle[i])).get_point();
    transformed_triangle[i] = reinterpret_cast<Vector3&>(vertex);
  }


  BeginMode3D(d->camera);

  DrawCube(Vector3(), 1.0f, 1.0f, 1.0f, WHITE);
  DrawCubeWires(Vector3(), 1.0f, 1.0f, 1.0f, MAROON);
  
  DrawTriangle3D(transformed_triangle[0], transformed_triangle[1], transformed_triangle[2], BLUE);
  DrawTriangle3D(transformed_triangle[0], transformed_triangle[2], transformed_triangle[1], BLUE);
  
  EndMode3D();
}


void quat_dquat_cleanup(void *data) {
  free(data);
}


