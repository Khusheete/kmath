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
#include <cstdlib>
#include <string>

#include "examples.hpp"
#include "raylib.h"
#include "raygui.h"


struct Test {
  std::string name;
  void *(*init)(void);
  void (*run)(void*);
  void (*cleanup)(void*);
};


const std::array<Test, 6> TESTS = {
  Test{
    .name = "Test Rotor3 and Motor3 structs",
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
    .name = "Motor3 Transforms",
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
  Test{
    .name = "OkLab interpolation",
    .init = &oklab_interpolation_init,
    .run = &oklab_interpolation_run,
    .cleanup = &oklab_interpolation_cleanup,
  },
  Test{
    .name = "Ease functions",
    .init = &ease_function_init,
    .run = &ease_function_run,
    .cleanup = &ease_function_cleanup,
  }
};


const Test *current_test = nullptr;
void *current_test_data = nullptr;


void test_selection_menu() {
  const int X_SIZE = 300;
  const int Y_SIZE = 32;
  const int Y_SPACING = 12;

  int render_width = GetRenderWidth();

  for (size_t i = 0; i < TESTS.size(); i++) {
    const Test &test = TESTS[i];
    const Rectangle button_bounds{
      .x = 0.5f * (render_width - X_SIZE),
      .y = float(i) * (Y_SIZE + Y_SPACING) + Y_SPACING,
      .width = X_SIZE,
      .height = Y_SIZE,
    };

    if (GuiButton(button_bounds, test.name.c_str())) {
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

  GuiLoadStyle("thirdparty/raygui/terminal/style_terminal.txt.rgs");

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(BLACK);

    if (current_test == nullptr) {
      test_selection_menu();
    } else {
      if (current_test_data == nullptr)
        current_test_data = current_test->init();

      current_test->run(current_test_data);

      if (IsKeyDown(KEY_BACKSPACE) || GuiButton(Rectangle{5.0, 5.0, 24.0, 24.0}, " < ")) {
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
