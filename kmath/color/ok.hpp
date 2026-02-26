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


#include "base.hpp"
#include "cie.hpp"


namespace kmath::ok {
  // Here are definitions and implementations of the ok color spaces by Björn Ottosson.
  // See: https://bottosson.github.io/posts/colorpicker/ and https://bottosson.github.io/posts/oklab/
  // Code for Ok color spaces is adapted from Björn Ottosson's blog


  // =========
  // = Types =
  // =========


  // In the following functions, OkHsl and OkHsv vectors are normalized between 0 and 1.
  // 
  // Technically speaking, OkLab vectors can be any arbitrary vector, though the lightness
  // value is usually between 0 and 1, and the ab values are between -0.5 and 0.5.
  // a is a range from green to red
  // b is a range from blue to yellow
  typedef _Vec3<float> OkLab;
  typedef _Vec3<float> OkHsl;
  typedef _Vec4<float> OkHsla;
  typedef _Vec3<float> OkHsv;
  typedef _Vec4<float> OkHsva;


  // ===================
  // = OkLab functions =
  // ===================
  

  OkLab xyzd65_to_oklab(const cie::XYZD65 &xyz);
  cie::XYZD65 oklab_to_xyzd65(const OkLab &lab);

  Lrgb oklab_to_lrgb(const OkLab &lab);
  OkLab lrgb_to_oklab(const Lrgb &rgb);
  

  inline Rgb oklab_to_rgb(const OkLab &lab) {
    return lrgb_to_rgb(oklab_to_lrgb(lab));
  }


  inline OkLab rgb_to_oklab(const Rgb &rgb) {
    return lrgb_to_oklab(rgb_to_lrgb(rgb));
  }


  // ===================
  // = OkHSV functions =
  // ===================


  OkLab okhsv_to_oklab(const OkHsv &hsv);
  OkHsv oklab_to_okhsv(const OkLab &lab);
  

  inline Lrgb okhsv_to_lrgb(const OkHsv &hsv) {
    return oklab_to_lrgb(okhsv_to_oklab(hsv));
  }


  inline OkHsv lrgb_to_okhsv(const Lrgb &rgb) {
    return oklab_to_okhsv(lrgb_to_oklab(rgb));
  }


  inline Rgb okhsv_to_rgb(const OkHsv &hsv) {
    return lrgb_to_rgb(okhsv_to_lrgb(hsv));
  }


  inline OkHsv rgb_to_okhsv(const Rgb &rgb) {
    return lrgb_to_okhsv(rgb_to_lrgb(rgb));
  }


  // ===================
  // = OkHSL functions =
  // ===================


  OkLab okhsl_to_oklab(const OkHsl &hsl);
  OkHsl oklab_to_okhsl(const OkLab &lab);


  inline Lrgb okhsl_to_lrgb(const OkHsl &hsl) {
    return oklab_to_lrgb(okhsl_to_oklab(hsl));
  }


  inline OkHsl lrgb_to_okhsl(const Lrgb &rgb) {
    return oklab_to_okhsl(lrgb_to_oklab(rgb));
  }


  inline Rgb okhsl_to_rgb(const OkHsl &hsl) {
    return lrgb_to_rgb(okhsl_to_lrgb(hsl));
  }


  inline OkHsl rgb_to_okhsl(const Rgb &rgb) {
    return lrgb_to_okhsl(rgb_to_lrgb(rgb));
  }
}
