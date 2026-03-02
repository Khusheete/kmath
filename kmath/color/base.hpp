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


#pragma once


#include "../vector.hpp"

#include <cstdint>


namespace kmath {
  // =========
  // = Types =
  // =========

  // Linear standard RGB
  typedef _Vec3<float> Lrgb;
  typedef _Vec4<float> Lrgba;
  // Non-linear standard RGB (gamma corrected)
  typedef _Vec3<float> Rgb;
  typedef _Vec4<float> Rgba;
  // HSL, HSV, and HWB color spaces based on RGB color space*s*.
  // Usually those color spaces are calculated from sRGB color spaces.
  // In functions implemented bellow, all components are normalized between 0 and 1 (inclusive).
  typedef _Vec3<float> Hsl;
  typedef _Vec3<float> Hsv;
  typedef _Vec3<float> Hwb;

  typedef _Vec3<uint8_t> RgbU8;
  typedef _Vec4<uint8_t> RgbaU8;

  

  // ======================
  // = Rgb-Srgb functions =
  // ======================


  // This is a widely used function to go from the lRGB color space to the sRGB space.
  inline float srgb_standard_gamma(const float value, const float gamma) {
    return (value >= 0.0f)? std::pow(value, 1.0f / gamma) : -std::pow(-value, 1.0f / gamma);
  }


  // This is a widely used function to go from the sRGB color space to the lRGB space.
  inline float srgb_standard_inv_gamma(const float value, const float gamma) {
    return (value >= 0.0f)? std::pow(value, gamma) : -std::pow(-value, gamma);
  }


  inline float srgb_standard_gamma(const float value) {
    return srgb_standard_gamma(value, 2.2f);
  }


  inline float srgb_standard_inv_gamma(const float value) {
    return srgb_standard_inv_gamma(value, 2.2f);
  }


  inline Rgb lrgb_to_rgb(const Lrgb &rgb, const float gamma) {
    return apply(rgb, [&](const float x) -> float { return srgb_standard_gamma(x, gamma); });
  }


  inline Lrgb rgb_to_lrgb(const Rgb &rgb, const float gamma) {
    return apply(rgb, [&](const float x) -> float { return srgb_standard_inv_gamma(x, gamma); });
  }


  inline Rgb lrgb_to_rgb(const Lrgb &rgb) {
    return lrgb_to_rgb(rgb, 2.2f);
  }


  inline Lrgb rgb_to_lrgb(const Rgb &rgb) {
    return rgb_to_lrgb(rgb, 2.2f);
  }


  inline Rgba lrgba_to_rgba(const Lrgba &rgba, const float gamma) {
    return Rgba(
      lrgb_to_rgb(*reinterpret_cast<const Vec3*>(&rgba), gamma),
      rgba.w
    );
  }


  inline Lrgba rgba_to_lrgba(const Rgba &rgba, const float gamma) {
    return Lrgba(
      rgb_to_lrgb(*reinterpret_cast<const Vec3*>(&rgba), gamma),
      rgba.w
    );
  }


  inline Rgba lrgba_to_rgba(const Lrgba &rgba) {
    return lrgba_to_rgba(rgba, 2.2f);
  }


  inline Lrgba rgba_to_lrgba(const Rgba &rgba) {
    return rgba_to_lrgba(rgba, 2.2f);
  }


  inline uint8_t float_to_u8(const float f) {
    return (f > 0.0f)? (f < 1.0f)? uint8_t(256.0f * f) : 255 : 0;
  }


  inline float u8_to_float(const uint8_t u8) {
    return float(u8) / 255.0f;
  }


  inline RgbU8 rgb_to_rgbu8(const Rgb &rgb) {
    return RgbU8(
      float_to_u8(rgb.x),
      float_to_u8(rgb.y),
      float_to_u8(rgb.z)
    );
  }


  inline Rgb rgbu8_to_rgb(const RgbU8 &rgb) {
    return Rgb(
      u8_to_float(rgb.x),
      u8_to_float(rgb.y),
      u8_to_float(rgb.z)
    );
  }


  // =================
  // = HSL functions =
  // =================

  
  Hsl rgb_to_hsl(const Rgb &rgb);
  Rgb hsl_to_rgb(const Hsl &hsl);


  inline Hsl lrgb_to_hsl(const Lrgb &rgb) {
    return rgb_to_hsl(lrgb_to_rgb(rgb));
  }


  inline Lrgb hsl_to_lrgb(const Hsl &hsl) {
    return rgb_to_lrgb(hsl_to_rgb(hsl));
  }


  // =================
  // = HSV functions =
  // =================


  Hsv rgb_to_hsv(const Rgb &rgb);
  Rgb hsv_to_rgb(const Hsv &hsv);

  inline Hsv lrgb_to_hsv(const Lrgb &rgb) {
    return rgb_to_hsv(lrgb_to_rgb(rgb));
  }


  inline Lrgb hsv_to_lrgb(const Hsv &hsv) {
    return rgb_to_lrgb(hsv_to_rgb(hsv));
  }


  // =================
  // = HWB functions =
  // =================


  Hwb hsv_to_hwb(const Hsv &hsv);
  Hsv hwb_to_hsv(const Hwb &hwb);


  inline Hwb lrgb_to_hwb(const Lrgb &rgb) {
    return hsv_to_hwb(lrgb_to_hsv(rgb));
  }


  inline Lrgb hwb_to_lrgb(const Hwb &hwb) {
    return hsv_to_lrgb(hwb_to_hsv(hwb));
  }


  inline Hwb rgb_to_hwb(const Rgb &rgb) {
    return hsv_to_hwb(rgb_to_hsv(rgb));
  }


  inline Rgb hwb_to_rgb(const Hwb &hwb) {
    return hsv_to_rgb(hwb_to_hsv(hwb));
  }
}
