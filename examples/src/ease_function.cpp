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

#include "kmath/kmath.hpp"
#include "kmath/interpolation_functions.hpp"

#include "thirdparty/raylib/raylib.h"
#include <array>
#include <cmath>


void *ease_function_init() {
  return nullptr;
}


void _draw_easing_func(const kmath::Vec2 &p_pos, kmath::ease::EasingFunction<double> p_ease, const char *p_name) {
  const int rect_size = 80;
  const int font_size = 25;
  
  Font default_font = GetFontDefault();
  Vector2 text_size = MeasureTextEx(default_font, p_name, font_size, 1.0);

  DrawText(
    p_name,
    p_pos.x - text_size.x / 2,
    p_pos.y - text_size.y / 2,
    font_size, WHITE
  );

  std::array<Vector2, 40> points;
  Vector2 rect_pos(p_pos.x - 0.5f * rect_size, p_pos.y + font_size);
  for (size_t i = 0; i < points.size(); i++) {
    float t = (float)i / (points.size() - 1);
    Vector2 pos((float)rect_size * t, (float)rect_size * p_ease(t));
    points[i] = Vector2(
      rect_pos.x + pos.x,
      rect_pos.y + rect_size - pos.y
    );
  }

  DrawLineStrip(points.data(), points.size(), SKYBLUE);
  // DrawRectangle(rect_pos.x, rect_pos.y, rect_size, rect_size, WHITE);
}


void ease_function_run([[maybe_unused]] void *_p_data) {
  using namespace kmath;
  using namespace kmath::ease;
  
  const std::array<EasingFunction<double>, 30> ease_funcs = {
    in::quad    , out::quad    , in_out::quad    ,
    in::cubic   , out::cubic   , in_out::cubic   ,
    in::quart   , out::quart   , in_out::quart   ,
    in::quint   , out::quint   , in_out::quint   ,
    in::sine    , out::sine    , in_out::sine    ,
    in::expo    , out::expo    , in_out::expo    ,
    in::circ    , out::circ    , in_out::circ    ,
    in::back    , out::back    , in_out::back    ,
    in::elastic , out::elastic , in_out::elastic ,
    in::bounce  , out::bounce  , in_out::bounce  ,
  };
  const std::array<const char*, 30> ease_func_names = {
    "Quad In"   , "Quad Out"   , "Quad In-Out"   ,
    "Cubic In"  , "Cubic Out"  , "Cubic In-Out"  ,
    "Quart In"  , "Quart Out"  , "Quart In-Out"  ,
    "Quint In"  , "Quint Out"  , "Quint In-Out"  ,
    "Sine In"   , "Sine Out"   , "Sine In-Out"   ,
    "Expo In"   , "Expo Out"   , "Expo In-Out"   ,
    "Circ In"   , "Circ Out"   , "Circ In-Out"   ,
    "Back In"   , "Back Out"   , "Back In-Out"   ,
    "Elastic In", "Elastic Out", "Elastic In-Out",
    "Bounce In" , "Bounce Out" , "Bounce In-Out" ,
  };

  int width = GetScreenWidth();

  const Vec2 spacing(180, 150);
  const Vec2 offset(spacing.x, 50.0);
  const int horizontal_count = width / spacing.x - 1;

  for (size_t i = 0; i < ease_funcs.size(); i++) {
    Vec2 position = offset + spacing * Vec2(i % horizontal_count, i / horizontal_count);
    _draw_easing_func(position, ease_funcs[i], ease_func_names[i]);
  }
}


void ease_function_cleanup([[maybe_unused]] void *_p_data) { }
