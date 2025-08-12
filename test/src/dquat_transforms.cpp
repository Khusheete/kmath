#include "kmath/kmath.hpp"
#include "tests.hpp"

#include "test/src/utils/freecam.hpp"
#include "test/src/utils/math.hpp"
#include "thirdparty/raylib/raylib.h"
#include <vector>


enum class TransformId : int {
  I, J, K, IE, JE, KE,
};


struct TestData {
  FreeCam camera;
  float prev_time;

  std::vector<kmath::Vec3f> reference_points;
  TransformId current_transform;
};


kmath::DQuatf get_transformation(TransformId p_trans) {
  switch (p_trans) {
  case TransformId::I: return kmath::DQuatf(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  case TransformId::J: return kmath::DQuatf(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  case TransformId::K: return kmath::DQuatf(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  case TransformId::IE: return kmath::DQuatf(0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0);
  case TransformId::JE: return kmath::DQuatf(0.0, 0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0);
  case TransformId::KE: return kmath::DQuatf(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0);
  }
  return kmath::DQuatf::IDENTITY;
}


TransformId &operator++(TransformId &p_trans) {
  p_trans = (TransformId)(1 + (int)p_trans);
  return p_trans;
}


void *dquat_transforms_init() {
  TestData *data = new TestData();
  data->camera.position = kmath::Vec3f(0.0, 0.5, -2.0);
  data->prev_time = GetTime();

  data->reference_points.reserve(10 * 10 * 10);
  for (int i = -5; i <= 5; ++i) {
    for (int j = -5; j <= 5; ++j) {
      for (int k = -5; k <= 5; ++k) {
        data->reference_points.push_back(kmath::Vec3f(
          i, j, k          
        ));
      }
    }
  }

  data->current_transform = TransformId::I;
  
  return data;
}



void dquat_transforms_run(void *p_data) {
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

  kmath::DQuatf current_transform = kmath::seplerp(
    kmath::DQuatf::IDENTITY,
    get_transformation(data->current_transform),
    ping_pong(0.5 * time)
  );
  for (int i = 0; i < data->reference_points.size(); ++i) {
    kmath::DQuatf transformed = kmath::DQuatf::from_point(data->reference_points[i]);
    transformed = current_transform.unit_conjugate(transformed);
    kmath::Vec3f new_pos = transformed.get_point();

    DrawPoint3D(reinterpret_cast<Vector3&>(new_pos), WHITE);
  }

  EndMode3D();
}


void dquat_transforms_cleanup(void *p_data) {
  delete (TestData*)p_data;
}


