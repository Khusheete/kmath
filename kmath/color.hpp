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
  typedef _Vec3<float> Lrgb;
  typedef _Vec4<float> Lrgba;
  // Non-linear standard RGB (gamma corrected)
  typedef _Vec3<float> Rgb;
  typedef _Vec4<float> Rgba;

  typedef _Vec3<uint8_t> RgbU8;
  typedef _Vec4<uint8_t> RgbaU8;

  typedef _Vec3<float> XYZD65;

  // See: https://bottosson.github.io/posts/colorpicker/ and https://bottosson.github.io/posts/oklab/
  // Code for Ok color spaces is adapted from Björn Ottosson's blog
  // 
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


  // =================
  // = XYZ functions =
  // =================
  

  inline XYZD65 lrgb_to_xyz(const Lrgb &rgb) {
    static const Mat3 transform(
      Vec3(0.4124f, 0.2126f, 0.0193f),
      Vec3(0.3576f, 0.7152f, 0.1192f),
      Vec3(0.1805f, 0.0722f, 0.9505f)
    );
    return transform * rgb;
  }


  inline Lrgb xyz_to_lrgb(const XYZD65 &xyz) {
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


  // This is a widely used function to go from the lRGB color space to the sRGB space.
  inline float srgb_standard_gamma(const float value, const float gamma = 2.2f) {
    return (value >= 0.0f)? std::pow(value, 1.0f / gamma) : -std::pow(-value, 1.0f / gamma);
  }


  // This is a widely used function to go from the sRGB color space to the lRGB space.
  inline float srgb_standard_inv_gamma(const float value, const float gamma = 2.2f) {
    return (value >= 0.0f)? std::pow(value, gamma) : -std::pow(-value, gamma);
  }


  inline Rgb lrgb_to_rgb(const Lrgb &rgb, const float gamma = 2.2f) {
    return apply(rgb, [&](const float x) -> float { return srgb_standard_gamma(x, gamma); });
  }


  inline Lrgb rgb_to_lrgb(const Rgb &rgb, const float gamma = 2.2f) {
    return apply(rgb, [&](const float x) -> float { return srgb_standard_inv_gamma(x, gamma); });
  }


  inline Rgba lrgba_to_rgba(const Lrgba &rgba, const float gamma = 2.2f) {
    return Rgba(
      lrgb_to_rgb(*reinterpret_cast<const Vec3*>(&rgba), gamma),
      rgba.w
    );
  }


  inline Lrgba rgba_to_lrgba(const Rgba &rgba, const float gamma = 2.2f) {
    return Lrgba(
      rgb_to_lrgb(*reinterpret_cast<const Vec3*>(&rgba), gamma),
      rgba.w
    );
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


  // ===================
  // = OkLab functions =
  // ===================
  

  OkLab xyz_to_oklab(const XYZD65 &xyz);
  XYZD65 oklab_to_xyz(const OkLab &lab);

  Lrgb oklab_to_lrgb(const OkLab &lab);
  OkLab lrgb_to_oklab(const Lrgb &rgb);
  

  inline Rgb oklab_to_rgb(const OkLab &lab) {
    return lrgb_to_rgb(oklab_to_lrgb(lab));
  }


  inline OkLab rgb_to_oklab(const Rgb &rgb) {
    return lrgb_to_oklab(lrgb_to_rgb(rgb));
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


  // ================================
  // = YCrCb/YCbCr Lab color spaces =
  // ================================


  // An implementation of the ITU-R BT 2020 (aka. Rec 2020) YCbCr standard.
  // See: https://www.itu.int/dms_pubrec/itu-r/rec/bt/R-REC-BT.2020-2-201510-I!!PDF-E.pdf
  namespace itu_bt_2020 {
    // Usual Y'CbCr with non-constant luminance
    typedef _Vec3<float> YCbCr;
    // Y'CrCb with non-constant luminance
    typedef _Vec3<float> YcCbcCrc;


    float gamma(const float value);
    float inv_gamma(const float value);


    Lrgb ycbcr_to_lrgb(const YCbCr &ycbcr);
    YCbCr lrgb_to_ycbcr(const Lrgb &lrgb);


    inline Rgb ycbcr_to_rgb(const YCbCr &ycbcr) {
      return lrgb_to_rgb(ycbcr_to_lrgb(ycbcr));
    }


    inline YCbCr rgb_to_ycbcr(const Rgb &rgb) {
      return lrgb_to_ycbcr(rgb_to_lrgb(rgb));
    }


    Lrgb yccbccrc_to_lrgb(const YcCbcCrc &ycbcr);
    YcCbcCrc lrgb_to_yccbccrc(const Lrgb &lrgb);


    inline Rgb yccbccrc_to_rgb(const YcCbcCrc &yccbccrc) {
      return lrgb_to_rgb(yccbccrc_to_lrgb(yccbccrc));
    }


    inline YcCbcCrc rgb_to_yccbccrc(const Rgb &rgb) {
      return lrgb_to_yccbccrc(rgb_to_lrgb(rgb));
    }
  }
}
