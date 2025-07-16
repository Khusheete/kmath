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
#include <iostream>

#include "kmath/kmath.hpp"
#include "thirdparty/raylib/raylib.h"


int main(void) {
  InitWindow(800, 600, "KMathTest");
  SetTargetFPS(60.0);

  Camera3D camera = {
    .position   = (Vector3){0.0f, 0.0f, 3.0f},
    .target     = (Vector3){0.0f, 0.0f, 0.0f},
    .up         = (Vector3){0.0f, 1.0f, 0.0f},
    .fovy       = 45.0f,
    .projection = CAMERA_PERSPECTIVE,
  };

  auto ping_pong = [](const double t) {
    return std::abs(std::fmod(t + 1.0, 2.0) - 1.0);
  };

  // kmath::Quat rot90 = kmath::Quat::from_axis_angle(kmath::Vec3::Y, 0.5 * std::numbers::pi);
  // Vector3 cube_position = {0.0f, 0.0f, 0.0f};
  
  std::array<kmath::Vec3, 3> triangle = {
    kmath::Vec3(0.0 ,  std::sqrt(3.0) / 4.0, 0.0),
    kmath::Vec3(-0.5, -std::sqrt(3.0) / 4.0, 0.0),
    kmath::Vec3(0.5 , -std::sqrt(3.0) / 4.0, 0.0),
  };

  kmath::DQuat start = kmath::DQuat::from_axis_angle_translation(kmath::Vec3::Y, 0.0, kmath::Vec3::X);
  kmath::DQuat end = kmath::DQuat::from_axis_angle_translation(kmath::Vec3::Y, std::numbers::pi, -kmath::Vec3::X);

  double prev_time = GetTime();
  while (!WindowShouldClose()) {
    double time = GetTime();
    // double delta = time - prev_time;

    // kmath::Quat rot = kmath::slerp(kmath::Quat::IDENTITY, rot90, 0.2 * std::numbers::pi * time);
    // kmath::Vec3 pos = (kmath::Vec3)kmath::conjugate_unit(rot, { 0.0, 0.0, 3.0, 0.0 });
    // camera.position.x = pos.x;
    // camera.position.y = pos.y;
    // camera.position.z = pos.z;
    
    kmath::DQuat transform = kmath::kenlerp(start, end, ping_pong(time), 0.7);
    std::array<Vector3, 3> transformed_triangle = {};

    for (int i = 0; i < 3; i++) {
      kmath::Vec3 vertex = kmath::conjugate_unit(transform, kmath::DQuat::from_point(triangle[i])).get_point();
      transformed_triangle[i].x = vertex.x;
      transformed_triangle[i].y = vertex.y;
      transformed_triangle[i].z = vertex.z;
    }

    // Draw the scene
    BeginDrawing();
    ClearBackground(BLACK);
    
    BeginMode3D(camera);

    // DrawCube(cube_position, 1.0f, 1.0f, 1.0f, WHITE);
    // DrawCubeWires(cube_position, 1.0f, 1.0f, 1.0f, MAROON);
    
    DrawTriangle3D(transformed_triangle[0], transformed_triangle[1], transformed_triangle[2], WHITE);
    DrawTriangle3D(transformed_triangle[0], transformed_triangle[2], transformed_triangle[1], WHITE);
    
    EndMode3D();

    EndDrawing();
  }

  CloseWindow();
  return EXIT_SUCCESS;
}
