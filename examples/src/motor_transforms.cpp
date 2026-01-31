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


#include "kmath/color.hpp"
#include "kmath/vector.hpp"
#include "kmath/motor_3d.hpp"
#include "examples.hpp"

#include "kmath/vector.hpp"
#include "utils/freecam.hpp"
#include "thirdparty/raylib/raylib.h"
#include <vector>


enum class TransformId : int {
  I, J, K, IE, JE, KE, MAX_ID
};


struct TestData {
  FreeCam camera;
  float prev_time;

  std::vector<kmath::Vec3> reference_points;
  TransformId current_transform;
};


kmath::Motor3 get_transformation(TransformId p_trans) {
  switch (p_trans) {
  case TransformId::I: return kmath::Motor3::from_axis_angle(kmath::Vec3::X, PI);
  case TransformId::J: return kmath::Motor3::from_axis_angle(kmath::Vec3::Y, PI);
  case TransformId::K: return kmath::Motor3::from_axis_angle(kmath::Vec3::Z, PI);
  case TransformId::IE: return kmath::Motor3::from_translation(4.0f * kmath::Vec3::X);
  case TransformId::JE: return kmath::Motor3::from_translation(4.0f * kmath::Vec3::Y);
  case TransformId::KE: return kmath::Motor3::from_translation(4.0f * kmath::Vec3::Z);
  case TransformId::MAX_ID: return kmath::Motor3::IDENTITY;
  }
  return kmath::Motor3::IDENTITY;
}


TransformId &operator++(TransformId &p_trans) {
  p_trans = (TransformId)((1 + (int)p_trans) % (int)TransformId::MAX_ID);
  return p_trans;
}


void *motor_transforms_init() {
  TestData *data = new TestData();
  data->camera.position = kmath::Vec3(0.0, 0.5, -2.0);
  data->prev_time = GetTime();

  data->reference_points.reserve(10 * 10 * 10);
  for (int i = -5; i <= 5; ++i) {
    for (int j = -5; j <= 5; ++j) {
      for (int k = -5; k <= 5; ++k) {
        data->reference_points.push_back(kmath::Vec3(
          i, j, k          
        ));
      }
    }
  }

  data->current_transform = TransformId::I;
  
  return data;
}



void motor_transforms_run(void *p_data) {
  TestData *data = (TestData*)p_data;

  // Get delta
  float time = GetTime();
  float delta = time - data->prev_time;
  data->prev_time = time;

  // Update freecam
  data->camera.update(delta);

  // Change current transform
  if (IsKeyPressed(KEY_TAB)) {
    ++data->current_transform;
  }


  // Draw scene
  data->camera.begin_mode_3d();

  DrawGrid(20, 1.0);

  kmath::Motor3 a = kmath::Motor3::IDENTITY; // kmath::Motor3::from_translation(4.0f * kmath::Vec3::X);
  kmath::Motor3 b = get_transformation(data->current_transform);

  kmath::Motor3 current_transform = kmath::lielerp(
    a, b,
    std::fmod(0.5f * time, 1.0f)
  );

  const kmath::Vec3 cube_size = 0.05f * kmath::Vec3::ONE;
  for (size_t i = 0; i < data->reference_points.size(); ++i) {
    kmath::Vec3 transformed = kmath::transform_point(data->reference_points[i], current_transform);
    transformed.y += 5.0f;

    const kmath::Lrgb c = (data->reference_points[i] + 5.0f * kmath::Vec3::ONE) / 10.0f;
    const kmath::RgbU8 cu8 = kmath::rgb_to_rgbu8(c);

    // const kmath::Vec3 draw_pos = transformed - 0.5f * cube_size;
    DrawCubeV(
      Vector3(transformed.x, transformed.y, transformed.z),
      Vector3(cube_size.x, cube_size.y, cube_size.z),
      {cu8.x, cu8.y, cu8.z, 255}
    );
  }

  EndMode3D();
}


void motor_transforms_cleanup(void *p_data) {
  delete (TestData*)p_data;
}


