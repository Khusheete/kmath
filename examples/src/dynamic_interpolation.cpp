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

#include "kmath/dynamic_interpolation.hpp"
#include "kmath/vector.hpp"

#include "raylib.h"
#include "raygui.h"
#include <cstdint>
#include <deque>


enum class Interpolator : uint8_t {
  EXPONENTIAL,
  SPRING,
  MAX,
};


static constexpr const float MAX_POSITION_LIST_TIME = 1.0; // Seconds


struct TestData {
  std::deque<kmath::Vec3> position_list;
  kmath::ExponentialInterpolator<float> exp_interpolator{1.0f};
  kmath::SpringInterpolator<float> spring_interpolator{1.0f, 1.0f};
  kmath::Vec2 position = kmath::Vec2::ZERO;
  kmath::Vec2 velocity = kmath::Vec2::ZERO;
  float prev_time;
  float target_fps = 120.0f;
  Interpolator interpolator = Interpolator::EXPONENTIAL;
  bool continuous_line = true;
};


void *dynamic_interpolation_init() {
  TestData *data = new TestData();
  data->prev_time = GetTime();
  return reinterpret_cast<void*>(data);
}


void dynamic_interpolation_run(void *p_data) {
  TestData *data = reinterpret_cast<TestData*>(p_data);

  // Get delta
  const float current_time = GetTime();
  const float delta = current_time - data->prev_time;
  data->prev_time = current_time;

  // Get mouse position
  const Vector2 rl_mouse_position = GetMousePosition();
  const kmath::Vec2 mouse_position(rl_mouse_position.x, rl_mouse_position.y);

  // Get screen dimentions
  const int render_width = GetRenderWidth();
  const int render_height = GetRenderHeight();
  const float gui_start_x = 0.1f * render_width;

  // Update frame
  switch (data->interpolator) {
  break;case Interpolator::EXPONENTIAL: {
    data->exp_interpolator.interpolate(data->position, mouse_position, delta);

    if (GuiButton(
        Rectangle{gui_start_x, 32.0f, 400.0f, 32.0f},
        "Exponential Interpolator"
      )) {
      data->interpolator = Interpolator::SPRING;
      data->velocity = kmath::Vec2::ZERO;
    }

    float characteristic_time = data->exp_interpolator.get_characteristic_time();
    const float base_characteristic_time = characteristic_time;
    GuiSlider(
      Rectangle{gui_start_x, 80.0f, 400.0f, 32.0f},
      "Characteristic time: ", "",
      &characteristic_time, 0.01f, 2.0f
    );

    if (characteristic_time != base_characteristic_time) {
      data->exp_interpolator.set_parameters(characteristic_time);
    }
  }
  break;case Interpolator::SPRING: {
    data->spring_interpolator.interpolate(data->position, data->velocity, mouse_position, delta);

    if (GuiButton(
        Rectangle{gui_start_x, 32.0f, 400.0f, 32.0f},
        "Spring Interpolator"
      )) {
      data->interpolator = Interpolator::EXPONENTIAL;
    }

    float stiffness = data->spring_interpolator.get_stiffness();
    float drag = data->spring_interpolator.get_drag();
    const float base_stiffness = stiffness;
    const float base_drag = drag;

    GuiSlider(
      Rectangle{gui_start_x, 80.0f, 400.0f, 32.0f},
      "Stiffness: ", "",
      &stiffness, 0.01f, 10.0f
    );
    GuiSlider(
      Rectangle{gui_start_x, 128.0f, 400.0f, 32.0f},
      "Drag: ", "",
      &drag, 0.01f, 10.0f
    );

    if (stiffness != base_stiffness || drag != base_drag) {
      data->spring_interpolator.set_parameters(stiffness, drag);
    }
  }
  break;case Interpolator::MAX: break;
  }

  const float base_fps = data->target_fps;
  GuiSlider(
    Rectangle{gui_start_x, render_height - 64.0f, 400.0f, 32.0f},
    "Target FPS: ", "",
    &data->target_fps, 10.0f, 128.0f
  );
  if (base_fps != data->target_fps) {
    SetTargetFPS(data->target_fps);
  }


  GuiCheckBox(
    Rectangle{render_width * 0.99f - 164.0f, render_height - 64.0f, 32.0f, 32.0f}, "Continuous Line",
    &data->continuous_line
  );


  // Draw the circle and its tail
  data->position_list.push_back(kmath::Vec3(data->position, current_time));

  while (current_time - data->position_list.front().z >= MAX_POSITION_LIST_TIME) {
    data->position_list.pop_front();
  }
  kmath::Vec2 prev_position = data->position_list.front().xy();
  for (kmath::Vec3 position : data->position_list) {
    DrawLine(prev_position.x, prev_position.y, position.x, position.y, BLUE);
    if (data->continuous_line) {
      prev_position = position.xy();
    }
  }

  DrawCircle(data->position.x, data->position.y, 10, WHITE);
}


void dynamic_interpolation_cleanup(void *p_data) {
  delete reinterpret_cast<TestData*>(p_data);
  SetTargetFPS(120);
}

