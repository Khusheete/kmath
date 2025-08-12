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
