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
#include "rotor_3d.hpp"
#include "utils.hpp"
#include "vector.hpp"

#include <cmath>
#include <cstdint>


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


  template<Number T>
  inline T angle_mod(const T angle, const T basis = T(TAU)) {
    const T base_mod = std::fmod(angle, basis);
    const T half_basis = T(0.5) * basis;
    return (base_mod < half_basis) ? (base_mod > -half_basis) ? base_mod : (base_mod + basis) : (base_mod - basis);
  }


  template<Number T>
  inline T angle_posmod(const T angle, const T basis = T(TAU)) {
    const T base_mod = std::fmod(angle, basis);
    return (base_mod >= T(0.0)) ? base_mod : (base_mod + basis);
  }


  template<Number T>
  inline T angle_difference(const T from, const T to, const T basis = T(TAU)) {
    const T diff = std::fmod(to - from, basis);
    return std::fmod(T(2) * diff, basis) - diff;
  }


  template<Number T>
  inline T angle_lerp(const T from, const T to, const T t, const T basis = T(TAU)) {
    return from + angle_difference(from, to, basis) * t;
  }


  template<Number T>
  inline T angle_inv_lerp(const T from, const T to, const T value, const T basis = T(TAU)) {
    const T base_diff = angle_difference(from, to, basis);
    const T mid_angle = from + base_diff * T(0.5);
    const T value_diff = angle_difference(mid_angle, value, basis);
    return value_diff / base_diff - T(0.5);
  }


  template<Number T>
  inline T angle_map(const T value, const T prev_min, const T prev_max, const T new_min, const T new_max, const T basis = T(TAU)) {
    return angle_lerp(new_min, new_max, angle_inv_lerp(prev_min, prev_max, value, basis), basis);
  }


  // This enum classifies euler basis with the following convention:
  // x, y, and z represent the axes of the original frame (describing extrinsic rotations)
  // X, Y, and Z represent the axes of the (mid-rotation) reference frame (describing intrinsic rotations)
  //
  // The classical "yaw, pitch, roll" euler angles are defined in the YXZ basis if your forward direction is along the Z axis.
  // They are ordered: (pitch, yaw, roll) in the euler angle vector.
  enum class EulerBasis: uint8_t {
    // Tait-Bryan Euler angles
    // In this representation, the three components of the euler angle vector represent rotation around their respective axis.
    // Only the order of the rotation changes.
    // Those representation are the most common for 3D engines and in engineering.
    XYZ,
    YZX,
    ZXY,
    XZY,
    ZYX,
    YXZ,
    zyx = XYZ,
    xzy = YZX,
    yxz = ZXY,
    yzx = XZY,
    xyz = ZYX,
    zxy = YXZ,

    // Classical Euler angles. This is the representation developped by Euler.
    // In this representation, the three components of the vector represent (in order) the angles to rotate around each specified axis.
    // For intrinsic rotations, the angles are used in this order: x then y then z.
    // For extrinsic rotations, the angles are used in this order: z then y then x.
    ZXZ,
    XYX,
    YZY,
    ZYZ,
    XZX,
    YXY,
    zxz = ZXZ,
    xyx = XYX,
    yzy = YZY,
    zyz = ZYZ,
    xzx = XZX,
    yxy = YXY,
  };


  template<Number T>
  _Mat3<T> euler_to_basis(const _Vec3<T> &rotation, const EulerBasis basis = EulerBasis::YXZ) {
    const _Vec3<T> c = apply(rotation, [](const T x) { return std::cos(x); });
    const _Vec3<T> s = apply(rotation, [](const T x) { return std::sin(x); });

    switch (basis) {
    break;case EulerBasis::xzy:
      return _Mat3<T>(
        _Vec3<T>(c.z * c.y, s.x * s.y + c.x * c.y * s.z, c.y * s.x * s.z - c.x * s.y),
        _Vec3<T>(-s.z     , c.x * c.z                  , c.z * s.x                  ),
        _Vec3<T>(c.z * s.y, c.x * s.z * s.y - c.y * s.x, c.x * c.y + s.x * s.z * s.y)
      );
    break;case EulerBasis::xyz:
      return _Mat3<T>(
        _Vec3<T>(c.y * c.z , c.x * s.z + c.z * s.x * s.y, s.x * s.z - c.x * c.z * s.y),
        _Vec3<T>(-c.y * s.z, c.x * c.z - s.x * s.y * s.z, c.z * s.x + c.x * s.y * s.z),
        _Vec3<T>(s.y       , - c.y * s.x                , c.x * c.y                  )
      );
    break;case EulerBasis::yxz:
      return _Mat3<T>(
        _Vec3<T>(c.y * c.z + s.y * s.x * s.z, c.x * s.z, c.y * s.x * s.z - c.z * s.y),
        _Vec3<T>(c.z * s.y * s.x - c.y * s.z, c.x * c.z, c.y * c.z * s.x + s.y * s.z),
        _Vec3<T>(c.x * s.y                  , -s.x     , c.y * c.x                  )
      );
    break;case EulerBasis::yzx:
      return _Mat3<T>(
        _Vec3<T>(c.y * c.z                  , s.z       , -c.z * s.y                 ),
        _Vec3<T>(s.y * s.x - c.y * c.x * s.z, c.z * c.x , c.y * s.x + c.x * s.y * s.z),
        _Vec3<T>(c.x * s.y + c.y * s.z * s.x, -c.z * s.x, c.y * c.x - s.y * s.z * s.x)
      );
    break;case EulerBasis::zyx:
      return _Mat3<T>(
        _Vec3<T>(c.z * c.y                  , c.y * s.z                  , -s.y     ),
        _Vec3<T>(c.z * s.y * s.x - c.x * s.z, c.z * c.x + s.z * s.y * s.x, c.y * s.x),
        _Vec3<T>(s.z * s.x + c.z * c.x * s.y, c.x * s.z * s.y - c.z * s.x, c.y * c.x)
      );
    break;case EulerBasis::zxy:
      return _Mat3<T>(
        _Vec3<T>(c.z * c.y - s.z * s.x * s.y, c.y * s.z + c.z * s.x * s.y, -c.x * s.y),
        _Vec3<T>(-c.x * s.z                 , c.z * c.x                  , s.x       ),
        _Vec3<T>(c.z * s.y + c.y * s.z * s.x, s.z * s.y - c.z * c.y * s.x, c.x * c.y)
      );

    break;case EulerBasis::xzx:
      return _Mat3<T>(
        _Vec3<T>(c.y, c.x * s.y, s.x * s.y                                           ),
        _Vec3<T>(-c.z * s.y, c.x * c.y * c.z - s.x * s.z, c.x * s.z + c.y * c.z * s.x),
        _Vec3<T>(s.y * s.z, -c.z * s.x - c.x * c.y * s.z, c.x * c.z - c.y * s.x * s.z)
      );
    break;case EulerBasis::xyx:
      return _Mat3<T>(
        _Vec3<T>(c.y      , c.x * s.y                   , s.a * s.y                  ),
        _Vec3<T>(s.y * s.z, c.x * c.z - c.y * s.x * s.z , c.z * s.x - c.x * c.y * s.z),
        _Vec3<T>(c.z * s.y, -c.x * s.z - c.y * c.z * s.x, c.x * c.y * c.z - s.x * s.z)
      );
    break;case EulerBasis::yxy:
      return _Mat3<T>(
        _Vec3<T>(c.x * c.z - c.y * s.x * s.z, s.y * s.z , -c.z * s.x - c.x * c.y * s.z),
        _Vec3<T>(s.x * s.y                  , c.y       , c.x * s.y                   ),
        _Vec3<T>(c.x * s.z + c.y * c.z * s.x, -c.z * s.y, c.x * c.y * c.z - s.x * s.z )
      );
    break;case EulerBasis::yzy:
      return _Mat3<T>(
        _Vec3<T>(c.x * c.y * c.z - s.x * s.z, c.y * s.y, -c.x * s.z - c.y * c.z * s.x),
        _Vec3<T>(-c.x * s.y                 , c.y      , s.x * s.y                   ),
        _Vec3<T>(c.z * s.x + c.x * c.y * s.z, s.y * s.z, c.x * c.z - c.y * s.x * s.z )
      );
    break;case EulerBasis::zyz:
      return _Mat3<T>(
        _Vec3<T>(c.x * c.y * c.z - s.x * s.z , c.x * s.z + c.y * c.z * s.x, -c.z * s.y),
        _Vec3<T>(-c.z * s.x - c.x * c.y * s.z, c.x * c.z - c.y * s.x * s.z, s.y * s.z ),
        _Vec3<T>(c.x * s.y                   , s.x * s.y                  , c.y       )
      );
    break;case EulerBasis::zxz:
      return _Mat3<T>(
        _Vec3<T>(c.x * c.z - c.y * s.x * s.z , c.z * s.x + c.x * c.y * s.z, s.y * s.z),
        _Vec3<T>(-c.x * s.z - c.y * c.z * s.x, c.x * c.y * c.z - s.x * s.z, c.z * s.y),
        _Vec3<T>(s.x * s.y                   , -c.x * s.y                 , c.y      )
      );
    }
  }


  template<Number T>
  _Vec3<T> basis_to_euler(const _Mat3<T> &rotation, const EulerBasis basis = EulerBasis::YXZ) {
    _Vec3<T> angles;

    switch (basis) {
    break;case EulerBasis::XZY:
      angles.x = std::atan2(rotation(2, 1), rotation(1, 1));
      angles.z = std::asin(-rotation(0, 1));
      angles.y = std::atan2(rotation(0, 2), rotation(0, 0));
    break;case EulerBasis::XYZ:
      angles.x = std::atan2(-rotation(1, 2), rotation(2, 2));
      angles.y = std::asin(rotation(0, 2));
      angles.z = std::atan2(-rotation(0, 1), rotation(0, 0));
    break;case EulerBasis::YXZ:
      angles.y = std::atan2(rotation(0, 2), rotation(2, 2));
      angles.x = std::asin(-rotation(1, 2));
      angles.z = std::atan2(rotation(1, 0), rotation(1, 1));
    break;case EulerBasis::YZX:
      angles.y = std::atan2(-rotation(2, 0), rotation(0, 0));
      angles.z = std::asin(rotation(1, 0));
      angles.x = std::atan2(-rotation(1, 2), rotation(1, 1));
    break;case EulerBasis::ZYX:
      angles.z = std::atan2(rotation(1, 0), rotation(0, 0));
      angles.y = std::asin(-rotation(2, 0));
      angles.x = std::atan2(rotation(2, 1), rotation(2, 2));
    break;case EulerBasis::ZXY:
      angles.z = std::atan2(-rotation(0, 1), rotation(1, 1));
      angles.x = std::asin(rotation(2, 1));
      angles.y = std::atan2(-rotation(2, 0), rotation(2, 2));

    break;case EulerBasis::XZX:
      angles.x = std::atan2(rotation(2, 0), rotation(1, 0));
      angles.y = std::acos(rotation(0, 0));
      angles.z = std::atan2(rotation(0, 2), -rotation(0, 1));
    break;case EulerBasis::XYX:
      angles.x = std::atan2(rotation(1, 0), -rotation(2, 0));
      angles.y = std::acos(rotation(0, 0));
      angles.z = std::atan2(rotation(0, 1), rotation(0, 2));
    break;case EulerBasis::YXY:
      angles.x = std::atan2(rotation(0, 1), rotation(2, 1));
      angles.y = std::acos(rotation(1, 1));
      angles.z = std::atan2(rotation(1, 0), -rotation(1, 2));
    break;case EulerBasis::YZY:
      angles.x = std::atan2(rotation(2, 1), -rotation(0, 1));
      angles.y = std::acos(rotation(1, 1));
      angles.z = std::atan2(rotation(1, 2), rotation(1, 0));
    break;case EulerBasis::ZYZ:
      angles.x = std::atan2(rotation(1, 2), rotation(0, 2));
      angles.y = std::acos(rotation(2, 2));
      angles.z = std::atan2(rotation(1, 0), -rotation(1, 2));
    break;case EulerBasis::ZXZ:
      angles.x = std::atan2(rotation(0, 2), -rotation(1, 2));
      angles.y = std::acos(rotation(2, 2));
      angles.z = std::atan2(rotation(2, 0), rotation(2, 1));
    }

    return angles;
  }


  template<Number T>
  _Rotor3<T> euler_to_rotor(const _Vec3<T> &euler, const EulerBasis basis = EulerBasis::YZX) {
    const _Vec3<T> half_angles = T(0.5) * euler;
    const _Vec3<T> c = apply(half_angles, [](const T x) { return std::cos(x); });
    const _Vec3<T> s = apply(-half_angles, [](const T x) { return std::sin(x); });

    switch (basis) {
    break;case EulerBasis::XYZ:
      return _Rotor3<T>(
         c.y * c.x * c.z - s.y * s.x * s.z,
        -s.y * c.x * s.z - c.y * s.x * c.z,
        -s.y * c.x * c.z + c.y * s.x * s.z,
        -c.y * c.x * s.z - s.y * s.x * c.z
      );
    break;case EulerBasis::YZX:
      return _Rotor3<T>(
        -s.x * s.z * s.y + c.x * c.z * c.y,
        -s.x * c.z * c.y - c.x * s.z * s.y,
        -c.x * c.z * s.y - s.x * s.z * c.y,
        -c.x * s.z * c.y + s.x * c.z * s.y
      );
    break;case EulerBasis::ZXY:
      return _Rotor3<T>(
         c.y * c.x * c.z - s.y * s.x * s.z,
         s.y * c.x * s.z - c.y * s.x * c.z,
        -s.y * c.x * c.z - c.y * s.x * s.z,
        -c.y * c.x * s.z - s.y * s.x * c.z
      );
    break;case EulerBasis::XZY:
      return _Rotor3<T>(
         s.x * s.z * s.y + c.y * c.x * c.z,
        -c.y * s.x * c.z + c.x * s.z * s.y,
         c.y * s.x * s.z - c.x * c.z * s.y,
        -s.x * c.z * s.y - c.y * c.x * s.z
      );
    break;case EulerBasis::ZYX:
      return _Rotor3<T>(
         s.x * s.z * s.y + c.x * c.z * c.y,
         c.x * s.z * s.y - s.x * c.z * c.y,
        -s.x * s.z * c.y - c.x * c.z * s.y,
        -c.x * s.z * c.y + s.x * c.z * s.y
      );
    break;case EulerBasis::YXZ:
      return _Rotor3<T>(
         s.x * s.z * s.y + c.x * c.z * c.y,
        -c.x * s.z * s.y - s.x * c.z * c.y,
        -c.x * c.z * s.y + s.x * s.z * c.y,
         s.x * c.z * s.y - c.x * s.z * c.y
      );

    break;case EulerBasis::ZXZ:
      return _Rotor3<T>(
        -c.y * s.x * s.z + c.y * c.x * c.z,
        -s.y * s.x * s.z - s.y * c.x * c.z,
        -s.y * s.x * c.z + s.y * c.x * s.z,
        -c.y * s.x * c.z - c.y * c.x * s.z
      );
    break;case EulerBasis::XYX:
      return _Rotor3<T>(
        -c.y * s.x * s.z + c.y * c.x * c.z,
        -c.y * s.x * c.z - c.y * c.x * s.z,
        -s.y * s.x * s.z - s.y * c.x * c.z,
        -s.y * s.x * c.z + s.y * c.x * s.z
      );
    break;case EulerBasis::YZY:
      return _Rotor3<T>(
         c.x * c.z * c.y - s.z * c.y * s.x,
        -c.z * s.y * s.x + c.x * s.z * s.y,
        -c.x * s.z * c.y - c.z * c.y * s.x,
        -c.x * c.z * s.y - s.z * s.y * s.x
      );
    break;case EulerBasis::ZYZ:
      return _Rotor3<T>(
        -c.y * s.x * s.z + c.z * c.y * c.x,
        -s.y * c.x * s.z + c.z * s.y * s.x,
        -c.z * s.y * c.x - s.y * s.x * s.z,
        -c.z * c.y * s.x - c.y * c.x * s.z
      );
    break;case EulerBasis::XZX:
      return _Rotor3<T>(
        -c.y * s.x * s.z + c.y * c.x * c.z,
        -c.y * c.x * s.z - c.y * s.x * c.z,
        -s.y * c.x * s.z + s.y * s.x * c.z,
        -s.y * s.x * s.z - s.y * c.x * c.z
      );
    break;case EulerBasis::YXY:
      return _Rotor3<T>(
        -s.z * c.y * s.x + c.x * c.z * c.y,
        -s.z * s.y * s.x - c.x * c.z * s.y,
        -c.x * s.z * c.y - c.z * c.y * s.x,
        -c.x * s.z * s.y + c.z * s.y * s.x
      );
    }
  }


  template<Number T>
  _Vec3<T> rotor_to_euler(const _Rotor3<T> &rotor, const EulerBasis basis = EulerBasis::YXZ) {
    return basis_to_euler(as_basis(rotor), basis);
  }
}
