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


namespace kmath::cie {
  // Here are definitions and implementation of color spaces as specified by the Commission Internationale de l'Éclairage (CIE)


  // =========
  // = Types =
  // =========

  typedef _Vec3<float> Lab;

  // The CIE XYZ tristimulus color space.
  // In functions bellow, the unit of the tristimulus XYZ is 1 (in other works, it is seldome 100).
  typedef _Vec3<float> XYZ;

  typedef _Vec3<float> xyY;
  // CIE xy chromaticity space. Corresponds to the first two components of the xyY color space.
  // Thus, this does not specify how dark some wolor is.
  typedef _Vec2<float> xy;


  // Identifies illuminants specified by CIE standards.
  enum class Illuminant {
    // Light of a incandescent tungsten filament at a temperature of 2856K.
    A,
    // Filtered version of illuminant A to emulate direct noon sunlight source. Correlated color temperature: 4874K.
    // This illuminant is obsolete. The D-series illuminants should be used instead.
    B,
    // Filtered version of illuminant A to emulate an average day light source. Correlated color temperature: 6774K.
    // This illuminant is obsolete. The D-series illuminants should be used instead.
    C,
    // Warm daylight source at sunrise or sunset. Correlated color temperature: 5003K.
    D50,
    // Mid-morning or mid-afternoon daylight source. Correlated color temperature: 5500K.
    D55,
    // Noon daylight source. Correlated color temperature: 6504K.
    // This illuminant is considered to be the standard illuminant. It is assumed for most color spaces, such as RGB, Rec2020, and OkLab.
    D65,
    // Northern sky daylight. Correlated color temperature: 7500K.
    D75,
    // High-efficiency blue phosphor monitor source. Used in BT.2035, and for Japan's NTSC-J analog televisions. Correlated color temperature: 9305K.
    D93,
    // Theoretical source with equal weight to all wavelengths.
    E,
    // Fluoressant light sources.
    FL1,
    FL2,
    FL3,
    FL4,
    FL5,
    FL6,
    FL7,
    FL8,
    FL9,
    FL10,
    FL11,
    FL12,
    FL3_1,
    FL3_2,
    FL3_3,
    FL3_4,
    FL3_5,
    FL3_6,
    FL3_7,
    FL3_8,
    FL3_9,
    FL3_10,
    FL3_11,
    FL3_12,
    FL3_13,
    FL3_14,
    FL3_15,
    // High pressure discharge lamps sources.
    HP1,
    HP2,
    HP3,
    HP4,
    HP5,
    // White LEDs sources.
    LED_B1,
    LED_B2,
    LED_B3,
    LED_B4,
    LED_B5,
    LED_BH1,
    LED_RGB1,
    LED_V1,
    LED_V2,
    // Natural indoor lighting: equivalent to D-series sources filtered through a window glass.
    ID50,
    ID65,
  };


  // =================
  // = XYZ functions =
  // =================
  

  XYZ lrgb_to_xyz(const Lrgb &rgb);
  Lrgb xyz_to_lrgb(const XYZ &xyz);


  inline XYZ rgb_to_xyz(const Rgb &rgb) {
    return lrgb_to_xyz(rgb_to_lrgb(rgb));
  }


  inline Rgb xyz_to_rgb(const XYZ &xyz) {
    return lrgb_to_rgb(xyz_to_lrgb(xyz));
  }


  // ========================
  // = xy and xyY functions =
  // ========================


  xyY xyz_to_xyy(const XYZ &xyz);
  XYZ xyy_to_xyz(const xyY &xyy);


  inline xyY lrgb_to_xyy(const Lrgb &rgb) {
    return xyz_to_xyy(lrgb_to_xyz(rgb));
  }


  inline Lrgb xyy_to_lrgb(const xyY &xyy) {
    return xyz_to_lrgb(xyy_to_xyz(xyy));
  }


  inline xyY rgb_to_xyy(const Rgb &rgb) {
    return xyz_to_xyy(rgb_to_xyz(rgb));
  }


  inline Rgb xyy_to_rgb(const xyY &xyy) {
    return xyz_to_rgb(xyy_to_xyz(xyy));
  }


  xy xyz_to_xy(const XYZ &xyz);


  inline xy lrgb_to_xy(const Lrgb &rgb) {
    return xyz_to_xy(lrgb_to_xyz(rgb));
  }

  
  inline xy rgb_to_xy(const Rgb &rgb) {
    return xyz_to_xy(rgb_to_xyz(rgb));
  }


  // ===============
  // = Illuminants =
  // ===============


  // Returns the chromaticity values for an illuminant for the 2° standard observer.
  xy illuminant_chromaticity(const Illuminant illum);

  // Returns the tristimulus value for an illuminant for the 2° standard observer for an assumed unit lightness (Y = 1).
  XYZ illuminant_tristimulus(const Illuminant illum);


  // ===================
  // = Lab color space =
  // ===================
  

  Lab xyz_to_lab(const XYZ &xyz, const Illuminant illum);
  XYZ lab_to_xyz(const Lab &lab, const Illuminant illum);


  // Conversion function if we use the D65 illuminant.
  inline Lab xyz_to_lab(const XYZ &xyz) {
    return xyz_to_lab(xyz, Illuminant::D65);
  }


  // Conversion function if we use the D65 illuminant.
  inline XYZ lab_to_xyz(const Lab &lab) {
    return lab_to_xyz(lab, Illuminant::D65);
  }


  inline Lab lrgb_to_lab(const Lrgb &rgb) {
    return xyz_to_lab(lrgb_to_xyz(rgb));
  }


  inline Lrgb lab_to_lrgb(const Lab &lab) {
    return xyz_to_lrgb(lab_to_xyz(lab));
  }


  inline Lab rgb_to_lab(const Rgb &rgb) {
    return xyz_to_lab(rgb_to_xyz(rgb));
  }


  inline Rgb lab_to_rgb(const Lab &lab) {
    return xyz_to_rgb(lab_to_xyz(lab));
  }
}
