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
#include <string>

#include "examples.hpp"
#include "thirdparty/raylib/raylib.h"


struct Test {
  std::string name;
  void *(*init)(void);
  void (*run)(void*);
  void (*cleanup)(void*);
};


const std::array<Test, 4> TESTS = {
  Test{
    .name = "Test Quat and DQuat structs",
    .init = &rotor_motor_init,
    .run = &rotor_motor_run,
    .cleanup = &rotor_motor_cleanup,
  },
  Test{
    .name = "Camera",
    .init = &camera_init,
    .run = &camera_run,
    .cleanup = &camera_cleanup,
  },
  Test{
    .name = "DQuat Transforms",
    .init = &motor_transforms_init,
    .run = &motor_transforms_run,
    .cleanup = &motor_transforms_cleanup,
  },
  Test{
    .name = "PGA Visualization",
    .init = &pga_visualization_init,
    .run = &pga_visualization_run,
    .cleanup = &pga_visualization_run,
  },
};


const Test *current_test = nullptr;
void *current_test_data = nullptr;


bool is_inside_rect(const Rectangle &p_rect, const Vector2 &p_point);
bool button(const char *p_text, const Vector2 &p_position, const float p_font_size, const Vector2 &p_inset);


void test_selection_menu() {
  const int Y_SIZE = 20;
  const int Y_SPACING = 28;
  const int FONT_SIZE = 26;

  int render_width = GetRenderWidth();

  Vector2 mouse_pos = GetMousePosition();

  for (int i = 0; i < TESTS.size(); i++) {
    const Test &test = TESTS[i];

    if (button(
      test.name.c_str(),
      Vector2(0.5f * render_width, i * (Y_SIZE + Y_SPACING) + Y_SPACING),
      FONT_SIZE,
      Vector2(10.0, 5.0))
    ) {
      current_test = &test;
    }
  }
}


int main(void) {
  InitWindow(800, 600, "KMathTest");
  SetWindowState(
    FLAG_WINDOW_RESIZABLE
  );
  SetTargetFPS(60.0);

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(BLACK);

    if (current_test == nullptr) {
      test_selection_menu();
    } else {
      if (current_test_data == nullptr)
        current_test_data = current_test->init();

      current_test->run(current_test_data);

      if (IsKeyDown(KEY_BACKSPACE) || button(" < ", Vector2(16.0, 16.0), 18.0, Vector2(5.0, 5.0))) {
        current_test->cleanup(current_test_data);
        current_test_data = nullptr;
        current_test = nullptr;
      }
    }

    EndDrawing();
  }

  CloseWindow();
  return EXIT_SUCCESS;
}


bool is_inside_rect(const Rectangle &p_rect, const Vector2 &p_point) {
  return p_rect.x <= p_point.x && p_point.x <= p_rect.x + p_rect.width && p_rect.y <= p_point.y && p_point.y <= p_rect.y + p_rect.height;
}


bool button(const char *p_text, const Vector2 &p_position, const float p_font_size, const Vector2 &p_inset) {
  Font default_font = GetFontDefault();
  Vector2 text_size = MeasureTextEx(default_font, p_text, p_font_size, 1.0);

  Rectangle button_rect = {
    p_position.x - 0.5f * text_size.x - p_inset.x,
    p_position.y - 0.5f * text_size.y - p_inset.y,
    text_size.x + 2.0f * p_inset.x,
    text_size.y + 2.0f * p_inset.y,
  };

  bool pressed = false;

  Color stroke_color = BLUE;
  Color fill_color = SKYBLUE;

  if (is_inside_rect(button_rect, GetMousePosition())) {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      stroke_color = RED;
      fill_color = GOLD;
    } else {
      stroke_color = LIME;
      fill_color = GREEN;
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
      pressed = true;
    }
  }

  DrawRectangleRec(button_rect, fill_color);
  DrawRectangleLinesEx(button_rect, 2.0, stroke_color);
  DrawTextEx(default_font, p_text, Vector2(button_rect.x + p_inset.x, button_rect.y + p_inset.y), p_font_size, 1.0, stroke_color);

  return pressed;
}
