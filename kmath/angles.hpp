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


#include "concepts.hpp"
#include "constants.hpp"
#include "vector.hpp"


namespace kmath {

  // Turns a vector in spherical coordinates (radius, polar, azimuth) to cartesian coordinates (Y up).
  template<Number T>
  _Vec3<T> spherical_to_cartesian(const _Vec3<T> &spherical) {
    return spherical.x * _Vec3<T>(
      std::cos(spherical.z) * std::sin(spherical.y),
      std::cos(spherical.y),
      std::sin(spherical.z) * std::sin(spherical.y)
    );
  }


  template<Number T>
  inline _Vec3<T> spherical_to_cartesian(const T radius, const T polar, const T azimuth) {
    return spherical_to_cartesian(_Vec3<T>(radius, polar, azimuth));
  }


  // Turns a vector in cartesian coordinates (Y up) to spherical coordinates (radius, polar, azimuth).
  template<Number T>
  _Vec3<T> cartesian_to_spherical(const _Vec3<T> &cartesian) {
    const T radius = length(cartesian);
    const _Vec3<T> unit = cartesian / radius;
    const T polar = std::acos(unit.y);
    const T azimuth = std::atan2(unit.z, unit.x);
    return _Vec3<T>(radius, polar, azimuth);
  }


  template<Number T>
  inline _Vec3<T> cartesian_to_spherical(const T x, const T y, const T z) {
    return cartesian_to_spherical(_Vec3<T>(x, y, z));
  }


  template<Number T>
  inline T degrees_to_radians(const T degree) {
    constexpr T conversion_coef = PI / 180.0;
    return degree * conversion_coef;
  }


  template<Number T>
  inline T radians_to_degrees(const T radians) {
    constexpr T conversion_coef = 180.0 / PI;
    return radians * conversion_coef;
  }

}
