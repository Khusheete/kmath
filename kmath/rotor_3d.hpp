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
#include "vector.hpp"
#include "matrix.hpp"
#include "euclidian_flat_3d.hpp"

#include <cmath>


namespace kmath {


  template<typename T>
  struct _Rotor3 {
    T s, e23, e31, e12;

  public:
    _Rotor3(): _Rotor3(IDENTITY) {}
    _Rotor3(T s, T e23, T e31, T e12): s(s), e23(e23), e31(e31), e12(e12) {}
    _Rotor3(T s, const _Vec3<T> &v): s(s), e23(v.x), e31(v.y), e12(v.z) {}


    static _Rotor3<T> from_axis_angle(const _Vec3<T> &axis, const T angle) {
      return _Rotor3<T>(
        (T)std::cos(angle / 2.0),
        (T)std::sin(angle / 2.0) * axis
      );
    }

  public:
    static const _Rotor3<T> ZERO;
    static const _Rotor3<T> IDENTITY;
  };


  template<typename T>
  const _Rotor3<T> _Rotor3<T>::ZERO = _Rotor3<T>((T)0.0, (T)0.0, (T)0.0, (T)0.0);
  template<typename T>
  const _Rotor3<T> _Rotor3<T>::IDENTITY = _Rotor3<T>((T)1.0, (T)0.0, (T)0.0, (T)0.0);


  // ===========================
  // = Rotor specific function =
  // ===========================


  template<typename T>
  KMATH_FUNC _Rotor3<T> reverse(const _Rotor3<T> &r) {
    return _Rotor3<T>(
      r.s, -r.e23, -r.e31, -r.e12
    );
  }


  template<typename T>
  KMATH_FUNC T length_squared(const _Rotor3<T> &r) {
    return r.s * r.s + r.e23 * r.e23 + r.e31 * r.e31 + r.e12 * r.e12;
  }


  template<typename T>
  KMATH_FUNC T length(const _Rotor3<T> &r) {
    return std::sqrt(length_squared(r));
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> normalized(const _Rotor3<T> &r) {
    return r / length(r);
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> inverse(const _Rotor3<T> &r) {
    return r / length_squared(r);
  }


  template<typename T>
  KMATH_FUNC _Vec3<T> get_direction(const _Rotor3<T> &r) {
    return _Vec3<T>(r.e23, r.e31, r.e12);
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> exp(const _Rotor3<T> &r) {
    T len_v = length(get_direction(r));
    T exp_w = std::exp(r.s);

    if (is_approx_zero(len_v)) return _Rotor3<T>(exp_w, exp_w * get_direction(r));

    return _Rotor3<T>(
      exp_w * std::cos(len_v),
      (exp_w * std::sin(len_v) / len_v) * get_direction(r)
    );
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> ln(const _Rotor3<T> &r) {
    T len = length(r);
    T len_v = length(_Vec3<T>(r.e23, r.e31, r.e12));

    if (is_approx_zero(len_v)) return _Rotor3<T>(std::log(len), _Vec3<T>::ZERO);
  
    return _Rotor3<T>(
      std::log(len),
      (std::acos(r.s / len) / len_v) * get_direction(r)
    );
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> pow(const _Rotor3<T> &r, const T power) {
    return exp(power * ln(r));
  }


  // ===================
  // = Rotor operators =
  // ===================

  
  template<typename T>
  KMATH_FUNC _Rotor3<T> slerp(const _Rotor3<T> &a, const _Rotor3<T> &b, const T t) {
    return a * pow(reverse(a) * b, t);
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> operator+(const _Rotor3<T> &a, const _Rotor3<T> &b) {
    _Rotor3<T> r(a);
    r += b;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> &operator+=(_Rotor3<T> &a, const _Rotor3<T> &b) {
    a.s   += b.s;
    a.e23 += b.e23;
    a.e31 += b.e31;
    a.e12 += b.e12;
    return a;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> operator-(const _Rotor3<T> &a, const _Rotor3<T> &b) {
    _Rotor3<T> r(a);
    r -= b;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> &operator-=(_Rotor3<T> &a, const _Rotor3<T> &b) {
    a.s   -= b.s;
    a.e23 -= b.e23;
    a.e31 -= b.e31;
    a.e12 -= b.e12;
    return a;    
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> operator-(const _Rotor3<T> &a) {
    return _Rotor3<T>(-a.s, -a.e23, -a.e31, -a.e12);
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> operator*(const T a, const _Rotor3<T> &b) {
    _Rotor3<T> r(b);
    r *= a;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> operator*(const _Rotor3<T> &a, const T b) {
    return b * a;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> &operator*=(_Rotor3<T> &a, const T b) {
    a.s   *= b;
    a.e23 *= b;
    a.e31 *= b;
    a.e12 *= b;
    return a;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> operator/(const _Rotor3<T> &a, const T b) {
    _Rotor3<T> r(a);
    r /= b;
    return r;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> &operator/=(_Rotor3<T> &a, const T b) {
    a.s   /= b;
    a.e23 /= b;
    a.e31 /= b;
    a.e12 /= b;
    return a;
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> operator*(const _Rotor3<T> &a, const _Rotor3<T> &b) {
    return _Rotor3<T>(
      -a.e23 * b.e23 - a.e31 * b.e31 - a.e12 * b.e12 + a.s   * b.s,
       a.s   * b.e23 + a.e23 * b.s + a.e31   * b.e12 - a.e12 * b.e31,
       a.s   * b.e31 + a.e31 * b.s + a.e12   * b.e23 - a.e23 * b.e12,
       a.s   * b.e12 + a.e12 * b.s + a.e23   * b.e31 - a.e31 * b.e23
    );
  }


  template<typename T>
  KMATH_FUNC _Rotor3<T> &operator*=(_Rotor3<T> &a, const _Rotor3<T> &b) {
    a = a * b;
    return a;
  }


  // ============
  // = As basis =
  // ============


  template<typename T>
  KMATH_FUNC _Vec3<T> get_x_basis_vector(const _Rotor3<T> &r) {
    return _Vec3<T>(
      r.e23 * r.e23 - r.e31 * r.e31 - r.e12 * r.e12 + r.s * r.s,
      2.0 * (r.e23 * r.e31 + r.e12 * r.s),
      2.0 * (r.e23 * r.e12 - r.e31 * r.s)
    );
  }


  template<typename T>
  KMATH_FUNC _Vec3<T> get_y_basis_vector(const _Rotor3<T> &r) {
    return _Vec3<T>(
      2.0 * (r.e23 * r.e31 - r.e12 * r.s),
      -r.e23 * r.e23 + r.e31 * r.e31 - r.e12 * r.e12 + r.s * r.s,
      2.0 * (r.e31 * r.e12 + r.e23 * r.s)
    );
  }


  template<typename T>
  KMATH_FUNC _Vec3<T> get_z_basis_vector(const _Rotor3<T> &r) {
    return _Vec3<T>(
      2.0 * (r.e23 * r.e12 + r.e31 * r.s),
      2.0 * (r.e31 * r.e12 - r.e23 * r.s),
      -r.e23 * r.e23 - r.e31 * r.e31 + r.e12 * r.e12 + r.s * r.s
    );
  }


  template<typename T>
  KMATH_FUNC _Mat3<T> get_basis(const _Rotor3<T> &r) {
    return _Mat3<T>(
      get_x_basis_vec(r),
      get_y_basis_vec(r),
      get_z_basis_vec(r)
    );
  }


  // ===================
  // = Transformations =
  // ===================
  

  template<typename T>
  KMATH_FUNC _Plane3<T> transform(const _Plane3<T> &a, const _Rotor3<T> &r);
  

  template<typename T>
  KMATH_FUNC _Line3<T> transform(const _Line3<T> &a, const _Rotor3<T> &r);


  template<typename T>
  KMATH_FUNC _Point3<T> transform(const _Point3<T> &a, const _Rotor3<T> &r);


  template<typename T>
  KMATH_FUNC _Vec3<T> transform(const _Vec3<T> &a, const _Rotor3<T> &r) {
    return _Vec3<T>(
      - a.x * r.e23 * r.e23 + a.x * r.e31 * r.e31 + a.x * r.e12 * r.e12 - a.x * r.s * r.s - (T)2.0 * a.y * r.s * r.e12 - (T)2.0 * a.y * r.e31 * r.e23 - (T)2.0 * a.z * r.e12 * r.e23 + (T)2.0 * a.z * r.s * r.e31,
      + a.y * r.e23 * r.e23 - a.y * r.e31 * r.e31 + a.y * r.e12 * r.e12 - a.y * r.s * r.s - (T)2.0 * a.z * r.s * r.e23 - (T)2.0 * a.z * r.e31 * r.e12 - (T)2.0 * a.x * r.e31 * r.e23 + (T)2.0 * a.x * r.s * r.e12,
      + a.z * r.e23 * r.e23 + a.z * r.e31 * r.e31 - a.z * r.e12 * r.e12 - a.z * r.s * r.s - (T)2.0 * a.x * r.e12 * r.e23 - (T)2.0 * a.x * r.s * r.e31 + (T)2.0 * a.y * r.s * r.e23 - (T)2.0 * a.y * r.e31 * r.e12
    );
  }


  // ================
  // = Type aliases =
  // ================
  

  typedef _Rotor3<float> Rotor3;
  typedef _Rotor3<double> Rotor3d;
  
  
  /*

  template<typename T>
  struct _Quat {
    // q = x . i + y . j + z . k + w
    T x, y, z, w;


    // Create a quaternion from an axis-angle rotation. axis must be a unit vector
    static _Quat<T> from_axis_angle(const _Vec3<T> &axis, const T angle) {
      return _Quat<T>(
        (T)std::sin(angle / 2.0) * axis,
        (T)std::cos(angle / 2.0)
      );
    }


    _Quat<T> conjugate() const {
      return _Quat<T>(-x, -y, -z, w);
    }


    // If this quaternion is a unit quaternion, prefer unit_conjugate
    _Quat<T> conjugate(const _Quat &q) const {
      return *this * q * inverse();
    }


    _Quat<T> unit_conjugate(const _Quat &q) const {
      return *this * q * conjugate();
    }


    _Quat<T> inverse(const _Quat<T> &a) {
      return conjugate() / length_squared();
    }


    _Quat<T> exp() {
      T len_v = ((_Vec3<T>*)this)->length();
      T exp_w = std::exp(w);

      if (is_approx_zero(len_v)) return { exp_w * *this, exp_w };

      return _Quat<T>(
        (exp_w * std::sin(len_v) / len_v) * (_Vec3<T>&)*this,
        exp_w * std::cos(len_v)
      );
    }


    _Quat<T> ln() {
      T len = length();
      T len_v = ((_Vec3<T>*)this)->length();

      if (is_approx_zero(len_v)) return _Quat<T>(_Vec3<T>::ZERO, std::log(len));
    
      return _Quat<T>(
        (std::acos(w / len) / len_v) * (_Vec3<T>&)*this,
        std::log(len)
      );
    }
    

    _Quat<T> pow(const T b) {
      return (b * ln()).exp();
    }


    T length_squared() const {
      return x * x + y * y + z * z + w * w;
    }


    T length() const {
      return std::sqrt(length_squared());
    }


    _Quat<T> normalize() const {
      _Quat<T> r(*this);
      r /= length();
      return r;
    }


    _Vec3<T> get_x_axis() const {
      return _Vec3<T>(
        x * x - y * y - z * z + w * w,
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w)
      );
    }


    _Vec3<T> get_y_axis() const {
      return _Vec3<T>(
        2.0 * (x * y - z * w),
        -x * x + y * y - z * z + w * w,
        2.0 * (y * z + x * w)
      );
    }


    _Vec3<T> get_z_axis() const {
      return _Vec3<T>(
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        -x * x - y * y + z * z + w * w
      );
    }


    _Mat3<T> get_basis() const {
      return _Mat3<T>(
        get_x_axis(),
        get_y_axis(),
        get_z_axis()
      );
    }


    operator const _Vec4<T>&() const {
      return *reinterpret_cast<const _Vec4<T>*>(this);
    }


    operator _Vec4<T>&() {
      return *reinterpret_cast<_Vec4<T>&>(this);
    }


    operator _Vec4<T>() const {
      return _Vec4<T>(x, y, z, w);
    }


    operator const _Vec3<T>&() const {
      return *reinterpret_cast<const _Vec3<T>*>(this);
    }


    operator _Vec3<T>&() {
      return *reinterpret_cast<_Vec3<T>*>(this);
    }


    operator _Vec3<T>() const {
      return _Vec3<T>(x, y, z);
    }


    _Quat(): x(0.0), y(0.0), z(0.0), w(1.0) {}
    _Quat(const T x, const T y, const T z, const T w) : x(x), y(y), z(z), w(w) {}
    _Quat(const _Vec3<T> &v, const T w) : x(v.x), y(v.y), z(v.z), w(w) {}
    _Quat(const _Vec3<T> &v) : x(v.x), y(v.y), z(v.z), w(0.0) {}

    static const _Quat<T> ZERO;
    static const _Quat<T> IDENTITY;
    static const _Quat<T> X;
    static const _Quat<T> Y;
    static const _Quat<T> Z;
    static const _Quat<T> W;
  };

  template<typename T>
  const _Quat<T> _Quat<T>::ZERO     = { 0.0, 0.0, 0.0, 0.0 };
  template<typename T>
  const _Quat<T> _Quat<T>::IDENTITY = { 0.0, 0.0, 0.0, 1.0 };
  template<typename T>
  const _Quat<T> _Quat<T>::X        = { 1.0, 0.0, 0.0, 0.0 };
  template<typename T>
  const _Quat<T> _Quat<T>::Y        = { 0.0, 1.0, 0.0, 0.0 };
  template<typename T>
  const _Quat<T> _Quat<T>::Z        = { 0.0, 0.0, 1.0, 0.0 };
  template<typename T>
  const _Quat<T> _Quat<T>::W        = { 0.0, 0.0, 0.0, 1.0 };


  template<typename T>
  _Quat<T> slerp(const _Quat<T> &a, const _Quat<T> &b, const double t) {
    return a * (a.conjugate() * b).pow(t);
  }


  template<typename T>
  _Quat<T> operator+(const _Quat<T> &a, const _Quat<T> &b) {
    _Quat<T> r(a);
    r += b;
    return r;
  }


  template<typename T>
  _Quat<T> &operator+=(_Quat<T> &a, const _Quat<T> &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
    return a;
  }


  template<typename T>
  _Quat<T> operator-(const _Quat<T> &a, const _Quat<T> &b) {
    _Quat<T> r(a);
    r -= b;
    return r;
  }


  template<typename T>
  _Quat<T> &operator-=(_Quat<T> &a, const _Quat<T> &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
    return a;    
  }


  template<typename T>
  _Quat<T> operator-(const _Quat<T> &a) {
    return _Quat<T>(-a.x, -a.y, -a.z, -a.w);
  }


  template<typename T>
  _Quat<T> operator*(const T a, const _Quat<T> &b) {
    _Quat<T> r(b);
    r *= a;
    return r;
  }


  template<typename T>
  _Quat<T> operator*(const _Quat<T> &a, const T b) {
    return b * a;
  }


  template<typename T>
  _Quat<T> &operator*=(_Quat<T> &a, const T b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
    return a;
  }


  template<typename T>
  _Quat<T> operator/(const _Quat<T> &a, const T b) {
    _Quat<T> r(a);
    r /= b;
    return r;
  }


  template<typename T>
  _Quat<T> &operator/=(_Quat<T> &a, const T b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
    return a;
  }


  template<typename T>
  _Quat<T> operator*(const _Quat<T> &a, const _Quat<T> &b) {
    return _Quat<T>(
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
      a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
      -a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w
    );
  }


  template<typename T>
  _Quat<T> &operator*=(_Quat<T> &a, const _Quat<T> &b) {
    a = a * b;
    return a;
  }

  */
  
}
