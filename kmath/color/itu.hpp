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


namespace kmath::itu {
  // Here are definitions and implementation of color spaces from the International Telecommunication Union (ITU).

  
  // An implementation of the ITU-R BT 2020 (aka. Rec 2020) YCbCr standard.
  // This standard defines color spaces for UHDTV with support for wide color gamut (WCG).
  // See: https://www.itu.int/dms_pubrec/itu-r/rec/bt/R-REC-BT.2020-2-201510-I!!PDF-E.pdf
  namespace bt_2020 {
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
