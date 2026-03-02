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


#include "base.hpp"

#include "../utils.hpp"

#include <algorithm>
#include <cmath>


namespace kmath {

  Hsl rgb_to_hsl(const Rgb &rgb) {
    const float max = std::max(std::max(rgb.x, rgb.y), rgb.z);
    const float min = std::min(std::min(rgb.x, rgb.y), rgb.z);
    Hsl hsl(
      0.0f, 0.0f, (min + max) * 0.5f
    );
    const float range = max - min;

    if (!is_approx_zero(range)) {
      hsl.y = (hsl.z == 0.0f || hsl.z == 1.0f)?
        0.0f
        : (max - hsl.z) / std::min(hsl.z, 1.0f - hsl.z);

      if (max == rgb.x) {
        hsl.x = (rgb.y - rgb.z) / range + (rgb.y < rgb.z? 6.0f : 0.0f);
      } else if (max == rgb.y) {
        hsl.x = (rgb.z - rgb.x) / range + 2.0f;
      } else {
        hsl.x = (rgb.x - rgb.y) / range + 4.0f;
      }

      hsl.x /= 6.0f;
    }


    if (hsl.y < 0.0f) {
      hsl.x += 0.5f;
      hsl.y = std::abs(hsl.y);
    }

    if (hsl.x >= 1.0f) {
      hsl.x -= 1.0f;
    }

    return hsl;
  }


  Rgb hsl_to_rgb(const Hsl &hsl) {
    const float a = hsl.y * std::min(hsl.z, 1.0f - hsl.z);
    const float k_r = 12.0f * std::fmod(hsl.x, 1.0f);
    const float k_g = 12.0f * std::fmod(2.0f / 3.0f + hsl.x, 1.0f);
    const float k_b = 12.0f * std::fmod(1.0f / 3.0f + hsl.x, 1.0f);

    return Rgb(
      hsl.z - a * std::max(-1.0f, std::min(std::min(k_r - 3.0f, 9.0f - k_r), 1.0f)),
      hsl.z - a * std::max(-1.0f, std::min(std::min(k_g - 3.0f, 9.0f - k_g), 1.0f)),
      hsl.z - a * std::max(-1.0f, std::min(std::min(k_b - 3.0f, 9.0f - k_b), 1.0f))
    );
  }


  Hsv rgb_to_hsv(const Rgb &rgb) {
    const float value = std::max(std::max(rgb.x, rgb.y), rgb.z);

    if (is_approx_zero(value)) {
      return Hsl(0.0f, 0.0f, 0.0f);
    }
    
    const float min = std::min(std::min(rgb.x, rgb.y), rgb.z);
    const float range = value - min;

    if (is_approx_zero(range)) {
      return Hsl(0.0f, 0.0f, value);
    }

    const float saturation = range / value;

    float hue;

    if (rgb.x == value) {
      hue = (rgb.y - rgb.z) / range;
    } else if (rgb.y == value) {
      hue = (rgb.z - rgb.x) / range + 2.0f;
    } else {
      hue = (rgb.x - rgb.y) / range + 4.0f;
    }

    hue /= 6.0f;
    if (hue < 0.0f) {
      hue += 1.0f;
    }

    return Hsv(hue, saturation, value);
  }


  Rgb hsv_to_rgb(const Hsv &hsv) {
    if (is_approx_zero(hsv.y)) {
      // Achromatic color
      return Rgb(hsv.z);
    }

    float six_h = std::fmod(6.0f * hsv.x, 6.0f);
    if (six_h < 0.0f) six_h += 6.0f;

    const int index = std::floor(six_h);
    const float h_fract = six_h - float(index);

    const float n = hsv.z * (1.0f - hsv.y);
    const float m = hsv.z * (1.0f - hsv.y * h_fract);
    const float k = hsv.z * (1.0f - hsv.y * (1.0f - h_fract));

    switch (index) {
    break;case 0: return Rgb(hsv.z, k    , n);
    break;case 1: return Rgb(m    , hsv.z, n);
    break;case 2: return Rgb(n    , hsv.z, k);
    break;case 3: return Rgb(n    , m    , hsv.z);
    break;case 4: return Rgb(k    , n    , hsv.z);
    break;case 5: return Rgb(hsv.z, n    , m);
    }
    return Rgb(); // This is dead code
  }


  Hwb hsv_to_hwb(const Hsv &hsv) {
    return Hwb(
      hsv.x,
      (1.0f - hsv.y) * hsv.z,
      1.0f - hsv.z
    );
  }


  Hsv hwb_to_hsv(const Hwb &hwb) {
    const float wb = hwb.y + hwb.z;
    float w = hwb.y;
    float b = hwb.z;
    if (wb > 1.0f) {
      const float inv_wb = 1.0f / wb;
      w *= inv_wb;
      b *= inv_wb;
    }
    const float rev_b = 1.0f - b;
    return Hsv(
      hwb.x,
      1.0f - w / rev_b,
      rev_b
    );
  }
};
