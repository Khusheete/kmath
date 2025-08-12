#include "freecam.hpp"
#include "thirdparty/raylib/raylib.h"


void FreeCam::update(float p_delta) {
  // Update camera position
  kmath::Vec3f forward = -direction.get_z_axis();
  kmath::Vec3f right = direction.get_x_axis();
  kmath::Vec3f up = direction.get_y_axis();

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

  position += 2.0f * movement_dir * p_delta;


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
    
    kmath::Quatf vertical_rotate = kmath::Quatf::from_axis_angle(right, -0.5 * mouse_delta.y * p_delta);
    kmath::Quatf horizontal_rotate = kmath::Quatf::from_axis_angle(kmath::Vec3f::Y, -0.5 * mouse_delta.x * p_delta);
    if (kmath::Vec3f::dot(kmath::Vec3f::Y, up) < 0.0) { // Inverse rotation if the camera's up vector is pointing down
      horizontal_rotate = horizontal_rotate.conjugate();
    }

    direction = (horizontal_rotate * vertical_rotate * direction).normalize();

    forward = -direction.get_z_axis();
    up = direction.get_y_axis();
  }

  // Update camera internals
  rl_camera.position = reinterpret_cast<Vector3&>(position);
  kmath::Vec3f camera_target = position + forward;
  rl_camera.target = reinterpret_cast<Vector3&>(camera_target);
  rl_camera.up = reinterpret_cast<Vector3&>(up);
}


void FreeCam::begin_mode_3d() const {
  BeginMode3D(rl_camera);
}
