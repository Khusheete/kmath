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
#include "utils/freecam.hpp"
#include "utils/pga.hpp"

#include "kmath/pga_3d.hpp"

#include "thirdparty/raylib/raylib.h"
#include <cstdlib>


struct TestData {
  FreeCam camera;
  float prev_time;
  kmath::Mvec3 some_point;
  kmath::Mvec3 some_line;
  kmath::Mvec3 some_plane;
};


void *pga_visualization_init() {
  TestData *data = new TestData();
  data->camera.position = kmath::Vec3(0.0, 1.0, 2.0);
  data->prev_time = GetTime();
  data->some_point = kmath::Mvec3::point(1.0, 1.0, 2.0);
  data->some_line = kmath::Mvec3::line(-1.0, 2.0, 5.0);
  data->some_plane = kmath::Mvec3::plane(1.0, -1.0, 0.0, 2.0).plane_normalize();
  
  return data;
}


void pga_visualization_run(void *p_data) {
  TestData *data = (TestData*)p_data;

  float time = GetTime();
  float delta = time - data->prev_time;
  data->prev_time = time;

  data->camera.update(delta);

  data->camera.begin_mode_3d();

  DrawGrid(10, 1.0);
  draw_point(data->some_point, RED);
  draw_line(data->camera.position, data->some_line, RED);
  draw_plane(data->camera.position, data->some_plane, BLUE);
  
  EndMode3D();
}


void pga_visualization_cleanup(void *p_data) {
  delete (TestData*)p_data;
}
