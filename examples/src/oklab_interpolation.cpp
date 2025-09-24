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


#include "examples.hpp"

#include "kmath/color.hpp"
#include "kmath/color.hpp"
#include "utils/freecam.hpp"

#include "thirdparty/raylib/raylib.h"

#include <cmath>
#include <array>


struct TestData {
  FreeCam camera;
  float prev_time;
};


void *oklab_interpolation_init() {
  TestData *data = new TestData();
  data->camera.position = 2.0f * kmath::Vec3::Z;
  data->prev_time = GetTime();
  return data;
}


void oklab_interpolation_run(void *p_data) {
  TestData *data = (TestData*)p_data;

  // Get delta
  float time = GetTime();
  float delta = time - data->prev_time;
  data->prev_time = time;

  // Update camera
  data->camera.update(delta);
  
  // Draw scene
  data->camera.begin_mode_3d();

  DrawGrid(20, 1.0);

  static const std::array<kmath::OkLab, 6> colors = {
    /* white  */ kmath::lrgb_to_oklab(kmath::Lrgb(1.0, 1.0, 1.0)),
    /* black  */ kmath::lrgb_to_oklab(kmath::Lrgb(0.0, 0.0, 0.0)),
    /* purple */ kmath::lrgb_to_oklab(kmath::Lrgb(0.5, 0.05, 0.5)),
    /* red    */ kmath::lrgb_to_oklab(kmath::Lrgb(1.0, 0.0, 0.0)),
    /* green  */ kmath::lrgb_to_oklab(kmath::Lrgb(0.0, 1.0, 0.0)),
    /* blue   */ kmath::lrgb_to_oklab(kmath::Lrgb(0.0, 0.0, 1.0))
  };

  static const std::array<kmath::Vec3, 6> col_positions = {
    kmath::Vec3(-10.0, -10.0, -10.0),
    kmath::Vec3(10.0, 10.0, 10.0),
    kmath::Vec3(-10.0, 10.0, 10.0),
    kmath::Vec3(10.0, -10.0, -10.0),
    kmath::Vec3(10.0, 10.0, -10.0),
    kmath::Vec3(-10.0, -10.0, 10.0)
  };


  for (int i = -20; i <= 20; i++) {
    for (int j = -20; j <= 20; j++) {
      for (int k = -20; k <= 20; k++) {
        const kmath::Vec3 pos = 0.5f * kmath::Vec3(i, j, k);

        std::array<float, colors.size()> weights;
        float total_weight = 0.0f;
        for (size_t index = 0; index < weights.size(); index++) {
          weights[index] = 1.0f / (1.0f + kmath::distance_squared(pos, col_positions[index]));
          total_weight += weights[index];
        }

        kmath::OkLab color;
        for (size_t index = 0; index < weights.size(); index++) {
          color += colors[index] * (weights[index] / total_weight);
        }

        kmath::RgbU8 lsrgb_color = kmath::as_rgbu8(kmath::lrgb_to_rgb(kmath::oklab_to_lrgb(color)));

        DrawPoint3D(
          Vector3(pos.x, pos.y, pos.z),
          Color(lsrgb_color.x, lsrgb_color.y, lsrgb_color.z, 255)
        );
      }
    }
  }
  
  EndMode3D();
}


void oklab_interpolation_cleanup(void *p_data) {
  delete (TestData*)p_data;
}

