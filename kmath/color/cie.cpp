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



#include "cie.hpp"

#include "../matrix.hpp"


namespace kmath::cie {
  // =================
  // = XYZ functions =
  // =================
  

  XYZ lrgb_to_xyz(const Lrgb &rgb) {
    static const Mat3 transform(
      Vec3(0.4124f, 0.2126f, 0.0193f),
      Vec3(0.3576f, 0.7152f, 0.1192f),
      Vec3(0.1805f, 0.0722f, 0.9505f)
    );
    return transform * rgb;
  }


  Lrgb xyz_to_lrgb(const XYZ &xyz) {
    static const Mat3 transform(
      Vec3(+3.2406f, -0.9689f, +0.0557f),
      Vec3(-1.5372f, +1.8758f, -0.2040f),
      Vec3(-0.4986f, +0.0415f, +1.0570f)
    );
    return transform * xyz;
  }


  // ========================
  // = xy and xyY functions =
  // ========================


  xyY xyz_to_xyy(const XYZ &xyz) {
    const float sum = xyz.x + xyz.y + xyz.z;
    return xyY(
      xyz.x / sum,
      xyz.y / sum,
      xyz.y
    );
  }


  XYZ xyy_to_xyz(const xyY &xyy) {
    return XYZ(
      xyy.z * xyy.x / xyy.y,
      xyy.z,
      xyy.z * (1.0f - xyy.x - xyy.y) / xyy.y
    );
  }


  xy xyz_to_xy(const XYZ &xyz) {
    const float sum = xyz.x + xyz.y + xyz.z;
    return xy(
      xyz.x / sum,
      xyz.y / sum
    );
  }


  // ===============
  // = Illuminants =
  // ===============


  xy illuminant_chromaticity(const Illuminant illum) {
    switch (illum) {
    case Illuminant::A: return xy(0.44758f, 0.40745f);
    case Illuminant::B: return xy(0.34842f, 0.35161f);
    case Illuminant::C: return xy(0.31006f, 0.31616f);
    case Illuminant::D50: return xy(0.34567f, 0.35850f);
    case Illuminant::D55: return xy(0.33242f, 0.34743f);
    case Illuminant::D65: return xy(0.31272f, 0.32903f);
    case Illuminant::D75: return xy(0.29902f, 0.31485f);
    case Illuminant::D93: return xy(0.28315f, 0.29711f);
    case Illuminant::E: return xy(0.33333f, 0.33333f);
    case Illuminant::FL1: return xy(0.31310f, 0.33727f);
    case Illuminant::FL2: return xy(0.37208f, 0.37529f);
    case Illuminant::FL3: return xy(0.40910f, 0.39430f);
    case Illuminant::FL4: return xy(0.44018f, 0.40329f);
    case Illuminant::FL5: return xy(0.31379f, 0.34531f);
    case Illuminant::FL6: return xy(0.37790f, 0.38835f);
    case Illuminant::FL7: return xy(0.31292f, 0.32933f);
    case Illuminant::FL8: return xy(0.34588f, 0.35875f);
    case Illuminant::FL9: return xy(0.37417f, 0.37281f);
    case Illuminant::FL10: return xy(0.34609f, 0.35986f);
    case Illuminant::FL11: return xy(0.38052f, 0.37713f);
    case Illuminant::FL12: return xy(0.43695f, 0.40441f);
    case Illuminant::FL3_1: return xy(0.4407f, 0.4033f);
    case Illuminant::FL3_2: return xy(0.3808f, 0.3734f);
    case Illuminant::FL3_3: return xy(0.3153f, 0.3439f);
    case Illuminant::FL3_4: return xy(0.4429f, 0.4043f);
    case Illuminant::FL3_5: return xy(0.3749f, 0.3672f);
    case Illuminant::FL3_6: return xy(0.3488f, 0.3600f);
    case Illuminant::FL3_7: return xy(0.4384f, 0.4045f);
    case Illuminant::FL3_8: return xy(0.3820f, 0.3832f);
    case Illuminant::FL3_9: return xy(0.3499f, 0.3591f);
    case Illuminant::FL3_10: return xy(0.3455f, 0.3560f);
    case Illuminant::FL3_11: return xy(0.3245f, 0.3434f);
    case Illuminant::FL3_12: return xy(0.4377f, 0.4037f);
    case Illuminant::FL3_13: return xy(0.4037f, 0.3724f);
    case Illuminant::FL3_14: return xy(0.3447f, 0.3609f);
    case Illuminant::FL3_15: return xy(0.3127f, 0.3290f);
    case Illuminant::HP1: return xy(0.533f, 0.415f);
    case Illuminant::HP2: return xy(0.4778f, 0.4158f);
    case Illuminant::HP3: return xy(0.4302f, 0.4075f);
    case Illuminant::HP4: return xy(0.3812f, 0.3797f);
    case Illuminant::HP5: return xy(0.3776f, 0.3713f);
    case Illuminant::LED_B1: return xy(0.4560f, 0.4078f);
    case Illuminant::LED_B2: return xy(0.4357f, 0.4012f);
    case Illuminant::LED_B3: return xy(0.3756f, 0.3723f);
    case Illuminant::LED_B4: return xy(0.3422f, 0.3502f);
    case Illuminant::LED_B5: return xy(0.3118f, 0.3236f);
    case Illuminant::LED_BH1: return xy(0.4474f, 0.4066f);
    case Illuminant::LED_RGB1: return xy(0.4557f, 0.4211f);
    case Illuminant::LED_V1: return xy(0.4548f, 0.4044f);
    case Illuminant::LED_V2: return xy(0.3781f, 0.3775f);
    case Illuminant::ID50: return xy(0.3432f, 0.3602f);
    case Illuminant::ID65: return xy(0.3107f, 0.3307f);
    }
    return xy(); // This should be dead code.
  }


  XYZ illuminant_tristimulus(const Illuminant illum) {
    return xyy_to_xyz(xyY(illuminant_chromaticity(illum), 1.0f));
  }


  // ===================
  // = Lab color space =
  // ===================


  Lab xyz_to_lab(const XYZ &xyz, const Illuminant illum) {
    const auto f = [](const float x) -> float {
      const float CUT = 0.008856451679035631f;
      return (x > CUT)? std::pow(x, 1.0f / 3.0f) : 7.833333333333333 * x + 0.13793103448275862f;
    };
    const XYZ illum_xyz = illuminant_tristimulus(illum);
    const Vec3 fxyz = apply(xyz / illum_xyz, f);
    return Lab(
      1.16f * fxyz.y - 0.16f,
      5.0f * (fxyz.x - fxyz.y),
      2.0f * (fxyz.y - fxyz.z)
    );
  }


  XYZ lab_to_xyz(const Lab &lab, const Illuminant illum) {
    const auto f = [](const float x) -> float {
      const float CUT = 0.20689655172413793f;
      return (x > CUT)? std::pow(x, 3) : (x - 0.13793103448275862f) * 0.12841854934601665f;
    };
    const XYZ illum_xyz = illuminant_tristimulus(illum);

    const float fy = (lab.x + 0.16f) * 0.8620689655172414f;
    const float fx = lab.y * 0.2f + fy;
    const float fz = fy - lab.z * 0.5f;

    return apply(Vec3(fx, fy, fz), f) * illum_xyz;
  }
}
