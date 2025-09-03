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
#include "test/src/utils/freecam.hpp"

#include "thirdparty/raylib/raylib.h"
#include <cmath>


struct TestData {
  FreeCam camera;
  float prev_time;
};


void *camera_init() {
  TestData *data = new TestData();
  data->camera.position = 2.0f * kmath::Vec3f::Z;
  data->prev_time = GetTime();
  return data;
}


void camera_run(void *p_data) {
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
  DrawCube(Vector3(), 1.0, 1.0, 1.0, WHITE);
  DrawCubeWires(Vector3(), 1.0, 1.0, 1.0, MAROON);

  EndMode3D();
}


void camera_cleanup(void *p_data) {
  delete (TestData*)p_data;
}
