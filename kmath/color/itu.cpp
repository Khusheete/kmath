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


#include "itu.hpp"


namespace kmath::itu {
  namespace bt_2020 {
    float gamma(const float value) {
      constexpr float alpha = 1.09929682680944f;
      constexpr float beta  = 0.01805396851080f;
      auto pos_gamma = [&](const float pos_value) -> float {
        return (pos_value < beta)?
          4.5f * pos_value
          : alpha * std::pow(pos_value, 0.45f) - (alpha - 1);
      };
      return (value >= 0.0f)? pos_gamma(value) : -pos_gamma(-value);
    }


    float inv_gamma(const float value) {
      constexpr float alpha = 1.09929682680944f;
      constexpr float beta  = 0.01805396851080f;
      constexpr float inv_alpha = 1.0f / alpha;
      constexpr float beta2 = 4.5f * beta;
      constexpr float a = 1.0f / 4.5f;
      constexpr float e = 1.0f / 0.45f;
      auto inv_pos_gamma = [&](const float pos_value) -> float {
        return (pos_value < beta2)?
          pos_value * a
          : std::pow(inv_alpha * (pos_value + (alpha - 1)), e);
      };
      return (value >= 0.0f)? inv_pos_gamma(value) : -inv_pos_gamma(-value);
    }

    
    Lrgb ycbcr_to_lrgb(const YCbCr &ycbcr) {
      const float bp = ycbcr.x + (ycbcr.y * 1.8814f);
      const float rp = ycbcr.x + (ycbcr.z * 1.4746f);
      const float gp = (ycbcr.x - 0.2627f * rp - 0.0593f * bp) / 0.6780f;
      return Rgb(inv_gamma(rp), inv_gamma(gp), inv_gamma(bp));
    }


    YCbCr lrgb_to_ycbcr(const Lrgb &lrgb) {
      const float rp = gamma(lrgb.x);
      const float gp = gamma(lrgb.y);
      const float bp = gamma(lrgb.z);
      const float y = 0.2627f * rp + 0.6780f * gp + 0.0593f * bp;
      const float cb = (bp - y) / 1.8814f;
      const float cr = (rp - y) / 1.4746f;
      return YCbCr(y, cb, cr);
    }


    Lrgb yccbccrc_to_lrgb(const YcCbcCrc &ycbcr) {
      constexpr float A = 0.2627f;
      constexpr float B = 0.6780f;
      constexpr float C = 0.0593f;

      constexpr float INV_B = 1.0f / B;

      constexpr float NB = -0.9702f;
      constexpr float PB =  0.7910f;
      constexpr float NR = -0.8591f;
      constexpr float PR =  0.4969f;

      constexpr float TWO_NB = -2.0f * NB;
      constexpr float TWO_PB =  2.0f * PB;
      constexpr float TWO_NR = -2.0f * NR;
      constexpr float TWO_PR =  2.0f * PR;

      const float bp = ycbcr.x
        + ((ycbcr.y <= 0.0f)?
          TWO_NB * ycbcr.y
          : TWO_PB * ycbcr.y);
      const float b = inv_gamma(bp);
      const float rp = ycbcr.x
        + ((ycbcr.z <= 0.0f)?
          TWO_NR * ycbcr.z
          : TWO_PR * ycbcr.z);
      const float r = inv_gamma(rp);
      const float y = inv_gamma(ycbcr.x);
      const float gb = INV_B * (y - A * r - C * b);
      const float g = inv_gamma(gb);

      return Lrgb(r, g, b);
    }


    YcCbcCrc lrgb_to_yccbccrc(const Lrgb &lrgb) {
      constexpr float A = 0.2627f;
      constexpr float B = 0.6780f;
      constexpr float C = 0.0593f;

      constexpr float NB = -0.9702f;
      constexpr float PB =  0.7910f;
      constexpr float NR = -0.8591f;
      constexpr float PR =  0.4969f;

      constexpr float INV_2NB = 1.0f / (-2.0f * NB);
      constexpr float INV_2PB = 1.0f / (2.0f * PB);
      constexpr float INV_2NR = 1.0f / (-2.0f * NR);
      constexpr float INV_2PR = 1.0f / (2.0f * PR);

      const float yc = gamma(A * lrgb.x + B * lrgb.y + C * lrgb.z);
      const float by_diff = gamma(lrgb.z) - yc;
      const float cbc = (by_diff <= 0.0f)?
        by_diff * INV_2NB
        : by_diff * INV_2PB;
      const float ry_diff = gamma(lrgb.x) - yc;
      const float crc = (ry_diff <= 0.0f)?
        ry_diff * INV_2NR
        : ry_diff * INV_2PR;

      return YcCbcCrc(yc, cbc, crc);
    }
  }
}
