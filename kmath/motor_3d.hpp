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


#include "defines.hpp"
#include "kmath/euclidian_flat_3d.hpp"
#include "kmath/utils.hpp"
#include "vector.hpp"
#include "matrix.hpp"
#include "rotor_3d.hpp"
#include <cmath>


namespace kmath {

  template<typename T>
  struct _Motor3 {
    T s, e23, e31, e12, e0123, e01, e02, e03;

  public:
    _Motor3(): _Motor3(IDENTITY) {}
    _Motor3(const T s, const T e23, const T e31, const T e12, const T e0123, const T e01, const T e02, const T e03): s(s), e23(e23), e31(e31), e12(e12), e0123(e0123), e01(e01), e02(e02), e03(e03) {}
    _Motor3(const _Rotor3<T> &real, const _Rotor3<T> &dual): s(real.s), e23(real.e23), e31(real.e31), e12(real.e12), e0123(dual.s), e01(dual.e23), e02(dual.e31), e03(dual.e12) {}


    static _Motor3<T> from_axis_angle(const _Vec3<T> &axis, const T angle) {
      return _Motor3<T>(
        _Rotor3<T>::from_axis_angle(axis, angle),
        _Rotor3<T>::ZERO
      );
    }
    

    static _Motor3<T> from_translation(const _Vec3<T> &translation) {
      return _Motor3<T>(
        _Rotor3<T>::IDENTITY,
        _Rotor3<T>((T)0.0, -((T)0.5) * translation)
      );
    }


    static _Motor3<T> from_rotor(const _Rotor3<T> &rotation) {
      return _Motor3<T>(rotation, _Rotor3<T>::ZERO);
    }


    static _Motor3<T> from_rotor_translation(const _Rotor3<T> &rotation, const _Vec3<T> &translation) {
      _Rotor3<T> trans((T)0.0, -((T)0.5) * translation);
      return _Motor3<T>(rotation, trans * rotation);
    }


    static _Motor3<T> from_axis_angle_translation(const _Vec3<T> &axis, const T angle, const _Vec3<T> translation) {
      _Rotor3<T> rot = _Rotor3<T>::from_axis_angle(axis, angle);
      _Rotor3<T> trans((T)0.0, -((T)0.5) * translation);
      return _Motor3<T>(rot, trans * rot);
    }

    
    static _Motor3<T> from_screw_coordinates(const _Vec3<T> &direction, const _Vec3<T> &moment, const T angle, const T translation) {
      if (!is_approx_zero(angle)) {
        T cos_a = std::cos(angle / 2.0);
        T sin_a = std::sin(angle / 2.0);
        return _Motor3<T>(
          _Rotor3<T>(cos_a, sin_a * direction),
          _Rotor3<T>((T)-0.5 * translation * sin_a, sin_a * moment + (T)(0.5 * translation * cos_a) * direction)
        );
      } else {
        return _Motor3<T>::from_translation(translation * direction);
      }
    }

  public:
    static const _Motor3<T> ZERO;
    static const _Motor3<T> IDENTITY;
  };


  template<typename T>
  const _Motor3<T> _Motor3<T>::ZERO = _Motor3<T>(_Rotor3<T>::ZERO, _Rotor3<T>::ZERO);
  template<typename T>
  const _Motor3<T> _Motor3<T>::IDENTITY = _Motor3<T>(_Rotor3<T>::IDENTITY, _Rotor3<T>::ZERO);


  // ===========================
  // = Motor specific function =
  // ===========================


  template<typename T>
  KMATH_FUNC const _Rotor3<T> &get_real_part(const _Motor3<T> &m) {
    return *reinterpret_cast<const _Rotor3<T>*>(&m);
  }


  template<typename T>
  KMATH_FUNC const _Rotor3<T> &get_dual_part(const _Motor3<T> &m) {
    return *(1 + reinterpret_cast<const _Rotor3<T>*>(&m));
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> get_rotor(const _Motor3<T> &m) {
    return get_real_part(m);
  }


  template<typename T>
  KMATH_FUNC _Vec3<T> get_translation(const _Motor3<T> &m) {
    const _Rotor3<T> &real = get_real_part(m);
    const _Rotor3<T> &dual = get_dual_part(m);
    _Rotor3<T> translation = ((T)2.0) * dual * reverse(real);
    return -_Vec3<T>(translation.e23, translation.e31, translation.e12);
  }


  // A motor is simple when its grade 4 part is null
  template<typename T>
  KMATH_FUNC bool is_simple(const _Motor3<T> &m) {
    return is_approx_zero(m.e0132);
  }


  // The fast square root returns the motor that does half the transformation as `m` modulo a positive factor
  template<typename T>
  KMATH_FUNC _Motor3<T> fast_sqrt(const _Motor3<T> &m) {
    T scaling = (T)1.0 + m.s;
    T half_g4 = (T)0.5 * m.e0123;
    return _Motor3<T>(
      scaling * scaling,
      scaling * m.e23,
      scaling * m.e31,
      scaling * m.e12,
      scaling * m.e0123 - m.s * half_g4,
      scaling * m.e01 + m.e23 * half_g4,
      scaling * m.e02 + m.e31 * half_g4,
      scaling * m.e03 + m.e12 * half_g4
    );
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> sqrt(const _Motor3<T> &m) {
    if (m.s >= 0.0) {
      T num = (T)2.0 * ((T)1.0 + m.s);
      T g4 = m.e0123 / num;
      return _Motor3<T>(
        (T)1.0 + m.s,
        m.e23,
        m.e31,
        m.e12,
        m.e0123 - m.s * g4,
        m.e01 + m.e23 * g4,
        m.e02 + m.e31 * g4,
        m.e03 + m.e12 * g4
      ) / std::sqrt(num);
    } else {
      T num = (T)2.0 * ((T)1.0 - m.s);
      T g4 = m.e0123 / num;
      return _Motor3<T>(
        (T)1.0 - m.s,
        -m.e23,
        -m.e31,
        -m.e12,
        -m.e0123 - m.s * g4,
        -m.e01 + m.e23 * g4,
        -m.e02 + m.e31 * g4,
        -m.e03 + m.e12 * g4
      ) / std::sqrt(num);
    }
  }


  template<typename T>
  KMATH_FUNC void to_screw_coordinates(const _Motor3<T> &m, _Vec3<T> &direction, _Vec3<T> &moment, T &angle, T &translation) {
    angle = 2.0 * std::acos(m.s);
    if (!is_approx_zero(angle)) {
      T inv_sin_a = std::sin(0.5 * angle);
      direction = inv_sin_a * _Vec3<T>(m.e23, m.e31, m.e12);
      translation = -2.0 * m.s * inv_sin_a;
      moment = inv_sin_a * _Vec3<T>(m.e01, m.e02, m.e03) - ((T)0.5) * translation * m.s * inv_sin_a * direction;
    } else {
      direction = get_translation(m);
      translation = length(direction);
      if (!is_approx_zero(translation)) { // Normalize translation direction
        direction /= translation;
      } else {
        direction = _Vec3<T>::ZERO;
      }
      moment = _Vec3<T>::INF;
    }
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> reverse(const _Motor3<T> &m) {
    return _Motor3<T>(
      m.s    , -m.e23, -m.e31, -m.e12,
      m.e0123, -m.e01, -m.e02, -m.e03
    );
  }


  template<typename T>
  KMATH_FUNC T magnitude_squared(const _Motor3<T> &m) {
    return m * reverse(m);
  }


  template<typename T>
  KMATH_FUNC T magnitude(const _Motor3<T> &m) {
    return std::sqrt(magnitude_squared(m));
  }
  

  template<typename T>
  KMATH_FUNC _Motor3<T> inverse(const _Motor3<T> &m) {
    return reverse(m) / magnitude_squared(m);
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> pow(const _Motor3<T> &m, T t) {
    _Vec3<T> direction, moment;
    T angle, translation;
    to_screw_coordinates(m, direction, moment, angle, translation);
    return _Motor3<T>::from_screw_coordinates(direction, moment, t * angle, t * translation);
  }


  template<typename T>
  KMATH_FUNC _Mat4<T> as_transform(const _Motor3<T> &m) {
    _Rotor3<T> rotor = get_rotor(m);
    _Vec3<T> translation = get_translation(m);
    _Vec3<T> rotation_x = get_x_basis_vector(rotor);
    _Vec3<T> rotation_y = get_y_basis_vector(rotor);
    _Vec3<T> rotation_z = get_z_basis_vector(rotor);
    return _Mat4<T>(
      _Vec4<T>(rotation_x.x, rotation_x.y, rotation_x.z, (T)0.0),
      _Vec4<T>(rotation_y.x, rotation_y.y, rotation_y.z, (T)0.0),
      _Vec4<T>(rotation_z.x, rotation_z.y, rotation_z.z, (T)0.0),
      _Vec4<T>(translation.x, translation.y, translation.z, (T)1.0)
    );
  }


  // ===================
  // = Motor operators =
  // ===================


  template<typename T>
  KMATH_FUNC _Motor3<T> operator+(const _Motor3<T> &a, const _Motor3<T> &b) {
    _Motor3<T> r(a);
    r += b;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> &operator+=(_Motor3<T> &a, const _Motor3<T> &b) {
    a.s     += b.s;
    a.e23   += b.e23;
    a.e31   += b.e31;
    a.e12   += b.e12;
    a.e0123 += b.e0123;
    a.e01   += b.e01;
    a.e02   += b.e02;
    a.e03   += b.e03;
    return a;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> operator-(const _Motor3<T> &a, const _Motor3<T> &b) {
    _Motor3<T> r(a);
    r -= b;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> &operator-=(_Motor3<T> &a, const _Motor3<T> &b) {
    a.s     -= b.s;
    a.e23   -= b.e23;
    a.e31   -= b.e31;
    a.e12   -= b.e12;
    a.e0123 -= b.e0123;
    a.e01   -= b.e01;
    a.e02   -= b.e02;
    a.e03   -= b.e03;
    return a;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> operator-(const _Motor3<T> &a) {
    return _Motor3<T>(
      -a.s, -a.e23, -a.e31, -a.e12, -a.e0123, -a.e01, -a.e02, -a.e03
    );
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> operator*(const T a, const _Motor3<T> &b) {
    _Motor3<T> r(b);
    r *= a;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> operator*(const _Motor3<T> &b, const T a) {
    return a * b;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> &operator*=(_Motor3<T> &a, const T b) {
    a.s     *= b;
    a.e23   *= b;
    a.e31   *= b;
    a.e12   *= b;
    a.e0123 *= b;
    a.e01   *= b;
    a.e02   *= b;
    a.e03   *= b;
    return a;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> operator/(const _Motor3<T> &a, const T b) {
    _Motor3<T> r(a);
    r /= b;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> &operator/=(_Motor3<T> &a, const T b) {
    a.s     /= b;
    a.e23   /= b;
    a.e31   /= b;
    a.e12   /= b;
    a.e0123 /= b;
    a.e01   /= b;
    a.e02   /= b;
    a.e03   /= b;
    return a;
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> operator*(const _Motor3<T> &a, const _Motor3<T> &b) {
    return _Motor3<T>(
      b.s * a.s - b.e23 * a.e23 - a.e31 * b.e31 - b.e12 * a.e12,
      a.s * b.e23 + b.s * a.e23 - a.e31 * b.e12 + b.e31 * a.e12,
      a.s * b.e31 + b.s * a.e31 + a.e23 * b.e12 - b.e23 * a.e12,
      a.s * b.e12 + b.s * a.e12 - a.e23 * b.e31 + b.e23 * a.e31,

      a.e01 * b.e23 + a.e02 * b.e31 + a.e03 * b.e12 + b.e01 * a.e23 + b.e02 * a.e31 + b.e03 * a.e12 + a.s * b.e0123 + b.s * a.e0123,
      a.s * b.e01 + b.s * a.e01 + a.e03 * b.e31 - a.e02 * b.e12 + b.e02 * a.e12 - b.e03 * a.e31 - a.e23 * b.e0123 - b.e23 * a.e0123,
      a.s * b.e02 + b.s * a.e02 + a.e01 * b.e12 - a.e03 * b.e23 - b.e01 * a.e12 + b.e03 * a.e23 - a.e31 * b.e0123 - b.e31 * a.e0123,
      a.s * b.e03 + b.s * a.e03 - a.e01 * b.e31 + a.e02 * b.e23 + b.e01 * a.e31 - b.e02 * a.e23 - a.e12 * b.e0123 - b.e12 * a.e0123
    );
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> &operator*=(_Motor3<T> &a, const _Motor3<T> &b) {
    a = a * b;
    return a;
  }
  

  // ===========================
  // = Interpolation functions =
  // ===========================


  template<typename T>
  KMATH_FUNC _Motor3<T> seplerp(const _Motor3<T> &a, const _Motor3<T> &b, const T t) {
    _Vec3<T> a_trans = get_translation(a);
    _Vec3<T> b_trans = get_translation(b);
    _Rotor3<T> a_rot = get_rotor(a);
    _Rotor3<T> b_rot = get_rotor(b);
    return _Motor3<T>::from_rotor_translation(slerp(a_rot, b_rot, t), lerp(a_trans, b_trans, t));
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> sclerp(const _Motor3<T> &a, const _Motor3<T> &b, const T t) {
    _Motor3<T> delta = reverse(a) * b;
    return a * pow(delta, t);
  }


  template<typename T>
  KMATH_FUNC _Motor3<T> kenlerp(const _Motor3<T> &a, const _Motor3<T> &b, const T t, const T beta) {
    _Motor3<T> sc_res = sclerp(a, b, t);
    _Rotor3<T> sc_rot = get_rotor(sc_res);
    _Vec3<T> sc_trans = get_translation(sc_res);
    _Motor3<T> sep_res = seplerp(a, b, t);
    _Rotor3<T> sep_rot = get_rotor(sep_res);
    _Vec3<T> sep_trans = get_translation(sep_res);
    return _Motor3<T>::from_rotor_translation(slerp(sc_rot, sep_rot, beta), lerp(sc_trans, sep_trans, beta));
  }
  

  // ===================
  // = Transformations =
  // ===================


  template<typename T>
  KMATH_FUNC _Plane3<T> transform(const _Plane3<T> &a, const _Motor3<T> &m) {
    return _Plane3<T>(
      - (T)2.0 * a.e1 * m.e02 * m.e12 - (T)2.0 * a.e2 * m.e03 * m.e23 - (T)2.0 * a.e3 * m.e01 * m.e31 + a.e0 * m.s * m.s + a.e0 * m.e23 * m.e23 + a.e0 * m.e31 * m.e31 + a.e0 * m.e12 * m.e12 + (T)2.0 * a.e1 * m.s * m.e01 + (T)2.0 * a.e2 * m.s * m.e02 + (T)2.0 * a.e3 * m.s * m.e03 + (T)2.0 * a.e1 * m.e0123 * m.e23 + (T)2.0 * a.e2 * m.e0123 * m.e31 + (T)2.0 * a.e3 * m.e0123 * m.e12 + (T)2.0 * a.e1 * m.e03 * m.e31 + (T)2.0 * a.e2 * m.e12 * m.e01 + (T)2.0 * a.e3 * m.e02 * m.e23,
      - a.e1 * m.e12 * m.e12 - a.e1 * m.e31 * m.e31 + a.e1 * m.s * m.s + a.e1 * m.e23 * m.e23 + (T)2.0 * a.e2 * m.e12 * m.s + (T)2.0 * a.e2 * m.e23 * m.e31 - (T)2.0 * a.e3 * m.s * m.e31 + (T)2.0 * a.e3 * m.e12 * m.e23,
      - a.e2 * m.e23 * m.e23 - a.e2 * m.e12 * m.e12 + a.e2 * m.s * m.s + a.e2 * m.e31 * m.e31 + (T)2.0 * a.e3 * m.s * m.e23 + (T)2.0 * a.e3 * m.e12 * m.e31 - (T)2.0 * a.e1 * m.s * m.e12 + (T)2.0 * a.e1 * m.e23 * m.e31,
      - a.e3 * m.e23 * m.e23 - a.e3 * m.e31 * m.e31 + a.e3 * m.s * m.s + a.e3 * m.e12 * m.e12 + (T)2.0 * a.e1 * m.s * m.e31 + (T)2.0 * a.e1 * m.e12 * m.e23 - (T)2.0 * a.e2 * m.s * m.e23 + (T)2.0 * a.e2 * m.e12 * m.e31
    );
  }
  

  template<typename T>
  KMATH_FUNC _Line3<T> transform(const _Line3<T> &a, const _Motor3<T> &m) {
    return _Line3<T>(
      - a.e23 * m.e31 * m.e31 - a.e23 * m.e12 * m.e12 + a.e23 * m.e23 * m.e23 + a.e23 * m.s * m.s + (T)2.0 * a.e31 * m.s * m.e12 - (T)2.0 * a.e12 * m.s * m.e31 + (T)2.0 * a.e31 * m.e23 * m.e31 + (T)2.0 * a.e12 * m.e12 * m.e23,
      - a.e31 * m.e23 * m.e23 - m.e12 * m.e12 * a.e31 + a.e31 * m.e31 * m.e31 + m.s * m.s * a.e31 - (T)2.0 * a.e23 * m.s * m.e12 + (T)2.0 * a.e12 * m.s * m.e23 + (T)2.0 * a.e12 * m.e12 * m.e31 + (T)2.0 * a.e23 * m.e23 * m.e31,
      - a.e12 * m.e23 * m.e23 - a.e12 * m.e31 * m.e31 + a.e12 * m.e12 * m.e12 + a.e12 * m.s * m.s + (T)2.0 * a.e23 * m.s * m.e31 - (T)2.0 * a.e31 * m.s * m.e23 + (T)2.0 * a.e23 * m.e12 * m.e23 + (T)2.0 * a.e31 * m.e12 * m.e31,
      - a.e01 * m.e31 * m.e31 - a.e01 * m.e12 * m.e12 + a.e01 * m.e23 * m.e23 + a.e01 * m.s * m.s - (T)2.0 * a.e12 * m.s * m.e02 - (T)2.0 * a.e03 * m.s * m.e31 - (T)2.0 * a.e23 * m.s * m.e0123 - (T)2.0 * a.e23 * m.e31 * m.e02 - (T)2.0 * a.e23 * m.e12 * m.e03 - (T)2.0 * a.e31 * m.e12 * m.e0123 + (T)2.0 * a.e31 * m.s * m.e03 + (T)2.0 * a.e02 * m.s * m.e12 + (T)2.0 * a.e03 * m.e12 * m.e23 + (T)2.0 * a.e02 * m.e23 * m.e31 + (T)2.0 * a.e31 * m.e01 * m.e31 + (T)2.0 * a.e31 * m.e23 * m.e02 + (T)2.0 * a.e12 * m.e03 * m.e23 + (T)2.0 * a.e12 * m.e12 * m.e01 + (T)2.0 * a.e23 * m.e23 * m.e01 + (T)2.0 * a.e12 * m.e0123 * m.e31,
      - a.e02 * m.e23 * m.e23 - a.e02 * m.e12 * m.e12 + a.e02 * m.e31 * m.e31 + a.e02 * m.s * m.s + (T)2.0 * a.e12 * m.s * m.e01 - (T)2.0 * a.e01 * m.s * m.e12 - (T)2.0 * a.e23 * m.s * m.e03 - (T)2.0 * a.e31 * m.e23 * m.e01 - (T)2.0 * a.e31 * m.e12 * m.e03 - (T)2.0 * a.e31 * m.s * m.e0123 - (T)2.0 * a.e12 * m.e0123 * m.e23 + (T)2.0 * a.e03 * m.s * m.e23 + (T)2.0 * a.e12 * m.e12 * m.e02 + (T)2.0 * a.e23 * m.e01 * m.e31 + (T)2.0 * a.e31 * m.e31 * m.e02 + (T)2.0 * a.e03 * m.e12 * m.e31 + (T)2.0 * a.e23 * m.e23 * m.e02 + (T)2.0 * a.e01 * m.e23 * m.e31 + (T)2.0 * a.e12 * m.e03 * m.e31 + (T)2.0 * a.e23 * m.e12 * m.e0123,
      - a.e03 * m.e23 * m.e23 - a.e03 * m.e31 * m.e31 + a.e03 * m.e12 * m.e12 + a.e03 * m.s * m.s - (T)2.0 * a.e02 * m.s * m.e23 - (T)2.0 * a.e31 * m.s * m.e01 + (T)2.0 * a.e23 * m.s * m.e02 - (T)2.0 * a.e12 * m.e23 * m.e01 - (T)2.0 * a.e12 * m.e31 * m.e02 - (T)2.0 * a.e12 * m.s * m.e0123 - (T)2.0 * a.e23 * m.e0123 * m.e31 + (T)2.0 * a.e01 * m.s * m.e31 + (T)2.0 * a.e23 * m.e12 * m.e01 + (T)2.0 * a.e31 * m.e12 * m.e02 + (T)2.0 * a.e12 * m.e12 * m.e03 + (T)2.0 * a.e01 * m.e12 * m.e23 + (T)2.0 * a.e02 * m.e12 * m.e31 + (T)2.0 * a.e23 * m.e03 * m.e23 + (T)2.0 * a.e31 * m.e03 * m.e31 + (T)2.0 * a.e31 * m.e0123 * m.e23
    );
  }


  template<typename T>
  KMATH_FUNC _Point3<T> transform(const _Point3<T> &a, const _Motor3<T> &m) {
    return _Point3<T>(
      - a.e032 * m.e31 * m.e31 - a.e032 * m.e12 * m.e12 + a.e032 * m.e23 * m.e23 + a.e032 * m.s * m.s - (T)2.0 * a.e021 * m.e31 * m.s - (T)2.0 * a.e123 * m.e01 * m.s - (T)2.0 * a.e123 * m.e02 * m.e12 - (T)2.0 * a.e123 * m.e0123 * m.e23 + (T)2.0 * a.e013 * m.e12 * m.s + (T)2.0 * a.e021 * m.e23 * m.e12 + (T)2.0 * a.e013 * m.e23 * m.e31 + (T)2.0 * a.e123 * m.e31 * m.e03,
      - a.e013 * m.e12 * m.e12 - a.e013 * m.e23 * m.e23 + a.e013 * m.e31 * m.e31 + a.e013 * m.s * m.s - (T)2.0 * a.e032 * m.e12 * m.s - (T)2.0 * a.e123 * m.e02 * m.s - (T)2.0 * a.e123 * m.e23 * m.e03 - (T)2.0 * a.e123 * m.e0123 * m.e31 + (T)2.0 * a.e021 * m.e23 * m.s + (T)2.0 * a.e032 * m.e23 * m.e31 + (T)2.0 * a.e021 * m.e31 * m.e12 + (T)2.0 * a.e123 * m.e01 * m.e12,
      - a.e021 * m.e23 * m.e23 - a.e021 * m.e31 * m.e31 + a.e021 * m.e12 * m.e12 + a.e021 * m.s * m.s - (T)2.0 * a.e013 * m.e23 * m.s - (T)2.0 * a.e123 * m.e03 * m.s - (T)2.0 * a.e123 * m.e01 * m.e31 - (T)2.0 * a.e123 * m.e0123 * m.e12 + (T)2.0 * a.e032 * m.e31 * m.s + (T)2.0 * a.e032 * m.e23 * m.e12 + (T)2.0 * a.e013 * m.e31 * m.e12 + (T)2.0 * a.e123 * m.e23 * m.e02,
      + a.e123 * (m.e23 * m.e23 - m.e31 * m.e31 - m.e12 * m.e12 - m.s * m.s)
    );
  }


  template<typename T>
  KMATH_FUNC _Vec3<T> transform_point(const _Vec3<T> &a, const _Motor3<T> &m) {
    T norm = m.s * m.s + m.e23 * m.e23 + m.e31 * m.e31 + m.e12 * m.e12;
    return _Vec3<T>(
      + a.x * m.e23 * m.e23 - a.x * m.e31 * m.e31 - a.x * m.e12 * m.e12 + a.x * m.s * m.s + (T)2.0 * a.y * m.e23 * m.e31 + (T)2.0 * a.y * m.s * m.e12 + (T)2.0 * a.z * m.e23 * m.e12 - (T)2.0 * a.z * m.s * m.e31 - (T)2.0 * m.e01 * m.s - (T)2.0 * m.e02 * m.e12 + (T)2.0 * m.e03 * m.e31 - (T)2.0 * m.e0123 * m.e23,
      - a.y * m.e23 * m.e23 + a.y * m.e31 * m.e31 - a.y * m.e12 * m.e12 + a.y * m.s * m.s + (T)2.0 * a.z * m.s * m.e23 + (T)2.0 * a.x * m.e23 * m.e31 + (T)2.0 * a.z * m.e31 * m.e12 - (T)2.0 * a.x * m.s * m.e12 + (T)2.0 * m.e01 * m.e12 - (T)2.0 * m.e02 * m.s - (T)2.0 * m.e03 * m.e23 - (T)2.0 * m.e0123 * m.e31,
      - a.z * m.e23 * m.e23 - a.z * m.e31 * m.e31 + a.z * m.e12 * m.e12 + a.z * m.s * m.s + (T)2.0 * a.x * m.e23 * m.e12 + (T)2.0 * a.x * m.e31 * m.s - (T)2.0 * a.y * m.e23 * m.s + (T)2.0 * a.y * m.e31 * m.e12 - (T)2.0 * m.e01 * m.e31 + (T)2.0 * m.e02 * m.e23 - (T)2.0 * m.e03 * m.s - (T)2.0 * m.e0123 * m.e12
    ) / norm;
  }


  template<typename T>
  KMATH_FUNC _Vec3<T> transform_direction(const _Vec3<T> &a, const _Motor3<T> &m) {
    return _Vec3<T>(
      + a.x * m.e23 * m.e23 - a.x * m.e31 * m.e31 - a.x * m.e12 * m.e12 + a.x * m.s * m.s + (T)2.0 * a.y * m.e31 * m.e23 + (T)2.0 * a.y * m.e12 * m.s - (T)2.0 * a.z * m.e31 * m.s + (T)2.0 * a.z * m.e12 * m.e23,
      - a.y * m.e23 * m.e23 + a.y * m.e31 * m.e31 - a.y * m.e12 * m.e12 + a.y * m.s * m.s + (T)2.0 * a.z * m.s * m.e23 + (T)2.0 * a.z * m.e31 * m.e12 + (T)2.0 * a.x * m.e31 * m.e23 - (T)2.0 * a.x * m.s * m.e12,
      - a.z * m.e23 * m.e23 - a.z * m.e31 * m.e31 + a.z * m.e12 * m.e12 + a.z * m.s * m.s + (T)2.0 * a.x * m.e12 * m.e23 + (T)2.0 * a.x * m.s * m.e31 - (T)2.0 * a.y * m.s * m.e23 + (T)2.0 * a.y * m.e31 * m.e12
    );
  }
  

  // ================
  // = Type aliases =
  // ================


  typedef _Motor3<float> Motor3;
  typedef _Motor3<double> Motor3d;
}
