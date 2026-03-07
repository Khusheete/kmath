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

#include "kmath/color/base.hpp"
#include "kmath/color/cie.hpp"
#include "kmath/color/ok.hpp"
#include "kmath/vector.hpp"
#include "kmath/print.hpp"

#include "raylib.h"
#include "raygui.h"

#include <iostream>


using namespace kmath;


enum class ColorSpace {
  OkHSL,
  OkHSV,
  OkHWB,
  OkLAB,
  HSL,
  HSV,
  HWB,
  XYZ,
  LAB,
  COLOR_SPACE_MAX,
};


ColorSpace &operator++(ColorSpace &p_color_space) {
  p_color_space = ColorSpace((int(p_color_space) + 1) % int(ColorSpace::COLOR_SPACE_MAX));
  return p_color_space;
}


struct TestData {
  ColorSpace csp;
  float value = 0.7f;
  bool convert_back = false;
};



void *color_spaces_init() {
  TestData *data = new TestData();
  return data;
}


// Returns a vector i
// i.x denotes the parameter that changes with TestData::value.
// i.yz denotes the parameters changing with the position inside the rectangle.
Vec3i get_color_space_permutation(const ColorSpace p_color_space) {
  switch (p_color_space) {
  case ColorSpace::OkHSL: 
  case ColorSpace::OkHSV:
  case ColorSpace::HSL:
  case ColorSpace::HSV:
    return Vec3i(2, 0, 1);
  case ColorSpace::OkLAB:
  case ColorSpace::LAB:
  case ColorSpace::OkHWB:
  case ColorSpace::HWB:
    return Vec3i(0, 1, 2);
  case ColorSpace::XYZ:
    return Vec3i(1, 0, 2);
  case ColorSpace::COLOR_SPACE_MAX:
    break;
  }
  return Vec3i(-1);
}


Vec3 get_color_space_min(const ColorSpace p_color_space) {
  switch (p_color_space) {
  case ColorSpace::OkHSL: 
  case ColorSpace::OkHSV:
  case ColorSpace::HSL:
  case ColorSpace::HSV:
  case ColorSpace::OkHWB:
  case ColorSpace::HWB:
  case ColorSpace::XYZ:
    return Vec3::ZERO;
  case ColorSpace::OkLAB:
    return Vec3(0.0f, -0.5f, -0.5f);
  case ColorSpace::LAB:
    return Vec3(0.0f, -1.0f, -1.0f);
  case ColorSpace::COLOR_SPACE_MAX:
    break;
  }
  return Vec3();
}


Vec3 get_color_space_max(const ColorSpace p_color_space) {
  switch (p_color_space) {
  case ColorSpace::OkHSL: 
  case ColorSpace::OkHSV:
  case ColorSpace::HSL:
  case ColorSpace::HSV:
  case ColorSpace::OkHWB:
  case ColorSpace::HWB:
  case ColorSpace::XYZ:
    return Vec3::ONE;
  case ColorSpace::OkLAB:
    return Vec3(1.0f, 0.5f, 0.5f);
  case ColorSpace::LAB:
    return Vec3::ONE;
  case ColorSpace::COLOR_SPACE_MAX:
    break;
  }
  return Vec3();
}


Vec3 build_vector(const Vec3 &p_coefs, const Vec3i &p_permutation, const Vec3 &p_min, const Vec3 &p_max) {
  Vec3 res{};
  res[p_permutation.x] = p_coefs.x;
  res[p_permutation.y] = p_coefs.y;
  res[p_permutation.z] = p_coefs.z;
  res = lerp(p_min, p_max, res);
  return res;
}


const char *get_color_space_name(const ColorSpace p_space) {
  switch (p_space) {
  case ColorSpace::OkHSL: return "OkHSL";
  case ColorSpace::OkHSV: return "OkHSV";
  case ColorSpace::OkHWB: return "OkHWB";
  case ColorSpace::OkLAB: return "OkLab";
  case ColorSpace::HSL:   return "HSL";
  case ColorSpace::HSV:   return "HSV";
  case ColorSpace::HWB:   return "HWB";
  case ColorSpace::XYZ:   return "XYZ";
  case ColorSpace::LAB:   return "Lab";
  case ColorSpace::COLOR_SPACE_MAX: return "ERROR";
  }
  return "ERROR";
}


typedef Vec3 (*ConvertFunc)(const Vec3 &);


ConvertFunc into_rgb_fun(const ColorSpace p_space) {
  switch (p_space) {
  case ColorSpace::OkHSL: return ok::okhsl_to_rgb;
  case ColorSpace::OkHSV: return ok::okhsv_to_rgb;
  case ColorSpace::OkHWB: return ok::okhwb_to_rgb;
  case ColorSpace::OkLAB: return ok::oklab_to_rgb;
  case ColorSpace::HSL: return hsl_to_rgb;
  case ColorSpace::HSV: return hsv_to_rgb;
  case ColorSpace::HWB: return hwb_to_rgb;
  case ColorSpace::XYZ: return cie::xyz_to_rgb;
  case ColorSpace::LAB: return cie::lab_to_rgb;
  case ColorSpace::COLOR_SPACE_MAX: break;
  }
  abort();
  return nullptr;
}


ConvertFunc from_rgb_fun(const ColorSpace p_space) {
  switch (p_space) {
  case ColorSpace::OkHSL: return ok::rgb_to_okhsl;
  case ColorSpace::OkHSV: return ok::rgb_to_okhsv;
  case ColorSpace::OkHWB: return ok::rgb_to_okhwb;
  case ColorSpace::OkLAB: return ok::rgb_to_oklab;
  case ColorSpace::HSL: return rgb_to_hsl;
  case ColorSpace::HSV: return rgb_to_hsv;
  case ColorSpace::HWB: return rgb_to_hwb;
  case ColorSpace::XYZ: return cie::rgb_to_xyz;
  case ColorSpace::LAB: return cie::rgb_to_lab;
  case ColorSpace::COLOR_SPACE_MAX: break;
  }
  abort();
  return nullptr;
}


void color_spaces_run(void *p_data) {
  TestData *data = reinterpret_cast<TestData*>(p_data);

  // Display color picker
  const int render_width = GetRenderWidth();
  const int render_height = GetRenderHeight();

  GuiSlider(
    Rectangle{render_width * 0.04f, 32.0f, render_width * 0.92f, 32.0f},
    "", "", &data->value, 0.0f, 1.0f
  );

  const Vec2 top_left{
    render_width * 0.04f,
    64.0f + 16.0f
  };
  const Vec2 bottom_right{
    render_width * 0.96f,
    render_height * 0.95f
  };
  const Vec2 size = bottom_right - top_left;

  const int dots_per_px = 10;
  const int horizontal_dots = size.x / dots_per_px;
  const int vertical_dots = size.y / dots_per_px;
  const float inv_horizontal_dots = 1.0f / horizontal_dots;
  const float inv_vertical_dots = 1.0f / vertical_dots;

  const Vec2 dot_size = size / Vec2(horizontal_dots, vertical_dots);

  const ConvertFunc into_rgb = into_rgb_fun(data->csp);
  const ConvertFunc from_rgb = from_rgb_fun(data->csp);

  const Vec3i permutation = get_color_space_permutation(data->csp);
  const Vec3 min_value = get_color_space_min(data->csp);
  const Vec3 max_value = get_color_space_max(data->csp);

  auto get_color = [&](const Vec3 &sij) -> Vec3 {
    return build_vector(sij, permutation, min_value, max_value);
  };

  for (int j = 0; j < vertical_dots; j++) {
    for (int i = 0; i < horizontal_dots; i++) {
      const Vec2 pos = top_left + Vec2(i, j) * dot_size;

      const Vec3 color = get_color(Vec3{data->value, i * inv_horizontal_dots, j * inv_vertical_dots});
      Rgb rgb_color = into_rgb(color);
      if (data->convert_back) {
        rgb_color = into_rgb(from_rgb(rgb_color)); // Convert into and from rgb to debug
      }
      const RgbU8 rgbu8_color = rgb_to_rgbu8(rgb_color);
      
      DrawRectangleRec(
        Rectangle{pos.x, pos.y, dot_size.x, dot_size.y},
        Color{rgbu8_color.x, rgbu8_color.y, rgbu8_color.z, 255}
      );
    }
  }

  // Change current color space
  for (int index = 0; index < int(ColorSpace::COLOR_SPACE_MAX); index++) {
    const ColorSpace csp = ColorSpace(index);
    const char *name = get_color_space_name(csp);
    bool checked = data->csp == csp;

    GuiCheckBox(
      Rectangle{float(48 + index * 128), 5, 24, 24},
      name,
      &checked
    );

    if (data->csp != csp && checked) {
      data->csp = csp;
    }
  }

  if (IsKeyPressed(KEY_TAB)) {
    ++data->csp;
  }

  // Identify color space
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    const Vec2 mouse_pos{
      float(GetMouseX()),
      float(GetMouseY()),
    };

    const Vec2 xy = (mouse_pos - top_left) / size;

    if (xy.x > 0.0f && xy.y > 0.0f && xy.x < 1.0f && xy.y < 1.0f) {
      const Vec3 color = get_color(Vec3{data->value, xy});
      const Vec3 rgb_color = into_rgb(color);
      const Vec3 back_move = from_rgb(rgb_color);
      const Vec3i colori{color * 100.0f};
      const Vec3i rgb_colori = Vec3i(rgb_to_rgbu8(rgb_color));

      std::cout << "Base Color: " << colori << std::endl;
      std::cout << "RGB: " << rgb_colori << std::endl;
      std::cout << "Convert Back Color: " << back_move << std::endl;
      std::cout << std::endl;
    }
  }

  // Add option to go through the back conversion function
  GuiCheckBox(
    Rectangle{
      render_width - 300.0f, render_height - 29.0f,
      24, 24
    },
    "Convert back to cylindrical space",
    &data->convert_back
  );
}


void color_spaces_cleanup(void *p_data) {
  delete reinterpret_cast<TestData*>(p_data);
}
