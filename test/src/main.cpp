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


#include <array>
#include <cmath>
#include <cstdlib>
#include <numbers>

#include "kmath/kmath.hpp"
#include "thirdparty/raylib/raylib.h"


int main(void) {
  InitWindow(800, 600, "KMathTest");
  SetTargetFPS(60.0);

  Camera3D camera = {
    .position   = (Vector3){0.0f, 0.0f, 5.0f},
    .target     = (Vector3){0.0f, 0.0f, 0.0f},
    .up         = (Vector3){0.0f, 1.0f, 0.0f},
    .fovy       = 45.0f,
    .projection = CAMERA_PERSPECTIVE,
  };

  auto ping_pong = [](const double t) {
    return std::abs(std::fmod(t + 1.0, 2.0) - 1.0);
  };

  kmath::Quatf rot90 = kmath::Quatf::from_axis_angle(kmath::Vec3f::Y, 0.5 * std::numbers::pi);
  Vector3 cube_position = {0.0f, 0.0f, 0.0f};
  
  std::array<kmath::Vec3f, 3> triangle = {
    kmath::Vec3f(0.0 ,  std::sqrt(3.0) / 4.0, 0.0),
    kmath::Vec3f(-0.5, -std::sqrt(3.0) / 4.0, 0.0),
    kmath::Vec3f(0.5 , -std::sqrt(3.0) / 4.0, 0.0),
  };

  kmath::DQuatf start = kmath::DQuatf::from_axis_angle_translation(kmath::Vec3f::Y, 0.0, 2.0f * kmath::Vec3f::X);
  kmath::DQuatf end = kmath::DQuatf::from_axis_angle_translation(kmath::Vec3f::Y, std::numbers::pi, -2.0f * kmath::Vec3f::X);

  double prev_time = GetTime();
  while (!WindowShouldClose()) {
    double time = GetTime();
    // double delta = time - prev_time;

    kmath::Quatf rot = kmath::slerp<float>(kmath::Quatf::IDENTITY, rot90, 0.2 * std::numbers::pi * time);
    kmath::Vec3f pos = (kmath::Vec3f)rot.unit_conjugate(kmath::Quatf(0.0, 0.0, 5.0, 0.0));
    camera.position = reinterpret_cast<Vector3&>(pos);
    
    kmath::DQuatf transform = kmath::kenlerp<float>(start, end, ping_pong(time), 0.7);
    std::array<Vector3, 3> transformed_triangle = {};

    for (int i = 0; i < 3; i++) {
      kmath::Vec3f vertex = transform.unit_conjugate(kmath::DQuatf::from_point(triangle[i])).get_point();
      transformed_triangle[i] = reinterpret_cast<Vector3&>(vertex);
    }

    // Draw the scene
    BeginDrawing();
    ClearBackground(BLACK);
    
    BeginMode3D(camera);

    DrawCube(cube_position, 1.0f, 1.0f, 1.0f, WHITE);
    DrawCubeWires(cube_position, 1.0f, 1.0f, 1.0f, MAROON);
    
    DrawTriangle3D(transformed_triangle[0], transformed_triangle[1], transformed_triangle[2], BLUE);
    DrawTriangle3D(transformed_triangle[0], transformed_triangle[2], transformed_triangle[1], BLUE);
    
    EndMode3D();

    EndDrawing();
  }

  CloseWindow();
  return EXIT_SUCCESS;
}
