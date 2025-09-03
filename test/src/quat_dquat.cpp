// Copyright © 2025 Souchet Ferdinand (aka. Khusheete)
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "tests.hpp"

#include "kmath/kmath.hpp"
#include "test/src/utils/math.hpp"

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


void quat_dquat_run(void *p_data) {
  TestData *data = (TestData*)p_data;

  float time = GetTime();
  
  kmath::Quatf rot = kmath::slerp<float>(kmath::Quatf::IDENTITY, data->camera_rotation, 0.2 * std::numbers::pi * time);
  kmath::Vec3f pos = (kmath::Vec3f)rot.unit_conjugate(kmath::Quatf(0.0, 0.0, 5.0, 0.0));
  data->camera.position = reinterpret_cast<Vector3&>(pos);
  
  kmath::DQuatf transform = kmath::kenlerp<float>(data->triangle_start, data->triangle_end, ping_pong(time), 0.7);
  std::array<Vector3, 3> transformed_triangle = {};

  for (int i = 0; i < 3; i++) {
    kmath::Vec3f vertex = transform.unit_conjugate(kmath::DQuatf::from_point(data->triangle[i])).get_point();
    transformed_triangle[i] = reinterpret_cast<Vector3&>(vertex);
  }


  BeginMode3D(data->camera);

  DrawCube(Vector3(), 1.0f, 1.0f, 1.0f, WHITE);
  DrawCubeWires(Vector3(), 1.0f, 1.0f, 1.0f, MAROON);
  
  DrawTriangle3D(transformed_triangle[0], transformed_triangle[1], transformed_triangle[2], BLUE);
  DrawTriangle3D(transformed_triangle[0], transformed_triangle[2], transformed_triangle[1], BLUE);
  
  EndMode3D();
}


void quat_dquat_cleanup(void *p_data) {
  delete (TestData*)p_data;
}


