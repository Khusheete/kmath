#include "tests.hpp"

#include "kmath/kmath.hpp"
#include "thirdparty/raylib/raylib.h"
#include <cmath>
#include <iostream>


struct TestData {
  Camera3D camera;
  kmath::Vec3f cam_position;
  kmath::Quatf cam_direction;
  float prev_time;
};


void *camera_init() {
  TestData *data = new TestData();
  data->camera = {
    .position = Vector3(),
    .target = Vector3(),
    .up = Vector3(0.0, 1.0, 0.0),
    .fovy = 70,
    .projection = CAMERA_PERSPECTIVE,
  };
  data->cam_position = 2.0f * kmath::Vec3f::Z;
  data->cam_direction = kmath::Quatf::IDENTITY;
  data->prev_time = GetTime();
  return data;
}


void camera_run(void *p_data) {
  TestData *data = (TestData*)p_data;

  // Get delta
  float time = GetTime();
  float delta = time - data->prev_time;
  data->prev_time = time;

  // Update camera position
  kmath::Vec3f forward = -data->cam_direction.get_z_axis();
  kmath::Vec3f right = data->cam_direction.get_x_axis();
  kmath::Vec3f up = data->cam_direction.get_y_axis();

  kmath::Vec3f movement_dir = kmath::Vec3f::ZERO;

  if (IsKeyDown(KEY_W)) {
    movement_dir += forward;
  }
  if (IsKeyDown(KEY_A)) {
    movement_dir -= right;
  }
  if (IsKeyDown(KEY_S)) {
    movement_dir -= forward;
  }
  if (IsKeyDown(KEY_D)) {
    movement_dir += right;
  }
  if (IsKeyDown(KEY_Q)) {
    movement_dir -= up;
  }
  if (IsKeyDown(KEY_E)) {
    movement_dir += up;
  }

  data->cam_position += 2.0f * movement_dir * delta;


  // Update camera rotation
  if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
    HideCursor();
  }
  if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
    ShowCursor();
  }
  
  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    float render_width = GetRenderWidth();
    float render_height = GetRenderHeight();
    
    Vector2 mouse_delta = GetMouseDelta();
    float mouse_mag = std::sqrt(mouse_delta.x * mouse_delta.x + mouse_delta.y * mouse_delta.y);
    SetMousePosition(0.5 * render_width, 0.5 * render_height);
    
    kmath::Quatf vertical_rotate = kmath::Quatf::from_axis_angle(right, -0.5 * mouse_delta.y * delta);
    kmath::Quatf horizontal_rotate = kmath::Quatf::from_axis_angle(kmath::Vec3f::Y, -0.5 * mouse_delta.x * delta);
    if (kmath::Vec3f::dot(kmath::Vec3f::Y, up) < 0.0) { // Inverse rotation if the camera's up vector is pointing down
      horizontal_rotate = horizontal_rotate.conjugate();
    }

    data->cam_direction = (horizontal_rotate * vertical_rotate * data->cam_direction).normalize();

    forward = -data->cam_direction.get_z_axis();
    up = data->cam_direction.get_y_axis();
  }

  // Update camera internals
  data->camera.position = reinterpret_cast<Vector3&>(data->cam_position);
  kmath::Vec3f camera_target = data->cam_position + forward;
  data->camera.target = reinterpret_cast<Vector3&>(camera_target);
  data->camera.up = reinterpret_cast<Vector3&>(up);


  // Draw scene
  BeginMode3D(data->camera);

  DrawGrid(20, 1.0);
  DrawCube(Vector3(), 1.0, 1.0, 1.0, WHITE);
  DrawCubeWires(Vector3(), 1.0, 1.0, 1.0, MAROON);

  EndMode3D();
}


void camera_cleanup(void *p_data) {
  delete (TestData*)p_data;
}
