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


#include "vector.hpp"
#include "matrix.hpp"

#include <cstdint>


namespace kmath {
  // =========
  // = Types =
  // =========

  // Linear standard RGB
  typedef _Vec3<float> Lsrgb;
  typedef _Vec4<float> Lsrgba;
  // Non-linear standard RGB (gamma corrected)
  typedef _Vec3<float> Srgb;
  typedef _Vec4<float> Srgba;

  typedef _Vec3<uint8_t> RgbU8;
  typedef _Vec4<uint8_t> RgbaU8;

  typedef _Vec3<float> XYZD65;

  // See: https://bottosson.github.io/posts/colorpicker/ and https://bottosson.github.io/posts/oklab/
  // Code for Ok color spaces is adapted from Björn Ottosson's blog
  typedef _Vec3<float> OkLab;
  typedef _Vec3<float> OkHsl;
  typedef _Vec4<float> OkHsla;
  typedef _Vec3<float> OkHsv;
  typedef _Vec4<float> OkHsva;


  // =================
  // = XYZ functions =
  // =================
  

  inline XYZD65 lrgb_to_xyz(const Lsrgb &rgb) {
    static const Mat3 transform(
      Vec3(0.4124f, 0.2126f, 0.0193f),
      Vec3(0.3576f, 0.7152f, 0.1192f),
      Vec3(0.1805f, 0.0722f, 0.9505f)
    );
    return transform * rgb;
  }


  inline Lsrgb xyz_to_lrgb(const XYZD65 &xyz) {
    static const Mat3 transform(
      Vec3(+3.2406f, -0.9689f, +0.0557f),
      Vec3(-1.5372f, +1.8758f, -0.2040f),
      Vec3(-0.4986f, +0.0415f, +1.0570f)
    );
    return transform * xyz;
  }
  

  // ======================
  // = Rgb-Srgb functions =
  // ======================


  inline Srgb lrgb_to_rgb(const Lsrgb &rgb, const float gamma = 2.2f) {
    return Srgb(
      (rgb.x >= 0.0f)? std::pow(rgb.x, 1.0f / gamma) : -std::pow(-rgb.x, 1.0f / gamma),
      (rgb.y >= 0.0f)? std::pow(rgb.y, 1.0f / gamma) : -std::pow(-rgb.y, 1.0f / gamma),
      (rgb.z >= 0.0f)? std::pow(rgb.z, 1.0f / gamma) : -std::pow(-rgb.z, 1.0f / gamma)
    );
  }


  inline Lsrgb rgb_to_lrgb(const Srgb &rgb, const float gamma = 2.2f) {
    return Lsrgb(
      (rgb.x >= 0.0f)? std::pow(rgb.x, gamma) : -std::pow(-rgb.x, gamma),
      (rgb.y >= 0.0f)? std::pow(rgb.y, gamma) : -std::pow(-rgb.y, gamma),
      (rgb.z >= 0.0f)? std::pow(rgb.z, gamma) : -std::pow(-rgb.z, gamma)
    );
  }


  inline Srgba lrgba_to_rgba(const Lsrgba &rgba, const float gamma = 2.2f) {
    return Srgba(
      lrgb_to_rgb(*reinterpret_cast<const Vec3*>(&rgba), gamma),
      rgba.w
    );
  }


  inline Lsrgba rgba_to_lrgba(const Srgba &rgba, const float gamma = 2.2f) {
    return Lsrgba(
      rgb_to_lrgb(*reinterpret_cast<const Vec3*>(&rgba), gamma),
      rgba.w
    );
  }


  inline RgbU8 as_rgbu8(const _Vec3<float> &rgb) {
    auto clamp = [](const float val){
      return (val >= 0.0)? (val <= 256.0)? val : 256.0 : 0.0;
    };
    return RgbU8(
      (uint8_t)clamp(256.0 * rgb.x),
      (uint8_t)clamp(256.0 * rgb.y),
      (uint8_t)clamp(256.0 * rgb.z)
    );
  }


  // ===================
  // = OkLab functions =
  // ===================
  

  OkLab xyz_to_oklab(const XYZD65 &xyz);
  XYZD65 oklab_to_xyz(const OkLab &lab);

  Lsrgb oklab_to_lrgb(const OkLab &lab);
  OkLab lrgb_to_oklab(const Lsrgb &rgb);
  

  // ===================
  // = OkHSV functions =
  // ===================


  OkLab okhsv_to_oklab(const OkHsv &hsv);
  OkHsv oklab_to_okhsv(const OkLab &lab);
  

  inline Lsrgb okhsv_to_lrgb(const OkHsv &hsv) {
    return oklab_to_lrgb(okhsv_to_oklab(hsv));
  }


  inline OkHsv lrgb_to_okhsv(const Lsrgb &rgb) {
    return oklab_to_okhsv(lrgb_to_oklab(rgb));
  }


  inline Srgb okhsv_to_rgb(const OkHsv &hsv) {
    return lrgb_to_rgb(okhsv_to_lrgb(hsv));
  }


  inline OkHsv rgb_to_okhsv(const Srgb &rgb) {
    return lrgb_to_okhsv(rgb_to_lrgb(rgb));
  }


  // ===================
  // = OkHSL functions =
  // ===================


  OkLab okhsl_to_oklab(const OkHsl &hsl);
  OkHsl oklab_to_okhsl(const OkLab &lab);


  inline Lsrgb okhsl_to_lrgb(const OkHsl &hsl) {
    return oklab_to_lrgb(okhsl_to_oklab(hsl));
  }


  inline OkHsl lrgb_to_okhsl(const Lsrgb &rgb) {
    return oklab_to_lrgb(lrgb_to_oklab(rgb));
  }


  inline Srgb okhsl_to_rgb(const OkHsl &hsl) {
    return lrgb_to_rgb(okhsl_to_lrgb(hsl));
  }


  inline OkHsl rgb_to_okhsl(const Srgb &rgb) {
    return lrgb_to_okhsl(rgb_to_lrgb(rgb));
  }
}
