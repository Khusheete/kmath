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


#include <ostream>
#include <cmath>


#ifndef KMATH_EPSILON
// Default epsilon value
#define KMATH_EPSILON 0.00001
#endif

#define KMATH_EPSILON2 (KMATH_EPSILON * KMATH_EPSILON)


namespace kmath {

  template<typename T>
  struct _Vec3;
  template<typename T>
  struct _Vec4;
  template<typename T>
  struct _Quat;
  template<typename T>
  struct _DQuat;
  template<typename T>
  struct _Mat3;
  template<typename T>
  struct _Mat4;


  typedef _Vec3<double> Vec3;
  typedef _Vec3<float> Vec3f;
  typedef _Vec3<int> Vec3i;
  typedef _Vec3<long> Vec3l;

  typedef _Vec4<double> Vec4;
  typedef _Vec4<float> Vec4f;
  typedef _Vec4<int> Vec4i;
  typedef _Vec4<long> Vec4l;

  typedef _Quat<double> Quat;
  typedef _Quat<float> Quatf;

  typedef _DQuat<double> DQuat;
  typedef _DQuat<float> DQuatf;

  typedef _Mat3<double> Mat3;
  typedef _Mat3<float> Mat3f;

  typedef _Mat4<double> Mat4;
  typedef _Mat4<float> Mat4f;
  

  // ==========================
  // = Utility math functions =
  // ==========================


  template<typename T>
  inline bool is_approx_zero(const T &a) {
    return a.length_squared() < KMATH_EPSILON2;
  }


  template<>
  inline bool is_approx_zero<double>(const double &a) {
    return std::abs(a) < KMATH_EPSILON;
  }


  template<>
  inline bool is_approx_zero<float>(const float &a) {
    return std::abs(a) < KMATH_EPSILON;
  }


  template<>
  inline bool is_approx_zero<int>(const int &a) {
    return a == 0;
  }


  template<>
  inline bool is_approx_zero<long>(const long &a) {
    return a == 0;
  }


  template<typename T>
  bool is_approx(const T &a, const T &b) {
    return is_approx_zero(b - a);
  }


  template<typename T>
  T inv_lerp(const T &a, const T &b, const T &x) {
    return (x - a) / (b - a);
  }


  template<typename T, typename S>
  T lerp(const T &a, const T &b, const S &t) {
    return (S)(1.0 - t) * a + t * b;
  }


  // ========
  // = Vec3 =
  // ========
  
  template<typename T>
  struct _Vec3 {
    T x, y, z;


    T length_squared() const {
      return x * x + y * y + z * z;
    }


    T length() const {
      return std::sqrt(length_squared());
    }


    _Vec3<T> normalize() const {
      _Vec3<T> r(*this);
      r /= length();
      return r;
    }


    static T dot(const _Vec3<T> &a, const _Vec3<T> &b) {
      return a.x * b.x + a.y * b.x + a.z * b.z;
    }


    _Vec3<T> cross(const _Vec3<T> &a, const _Vec3<T> &b) {
      return _Vec3<T>(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
      );
    }


    static const _Vec3<T> ZERO;
    static const _Vec3<T> ONE;
    static const _Vec3<T> INF;
    static const _Vec3<T> X;
    static const _Vec3<T> Y;
    static const _Vec3<T> Z;
  };

  template<typename T>
  const _Vec3<T> _Vec3<T>::ZERO = { 0.0, 0.0, 0.0 };
  template<typename T>
  const _Vec3<T> _Vec3<T>::ONE  = { 1.0, 1.0, 1.0};
  template<typename T>
  const _Vec3<T> _Vec3<T>::INF  = { std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity() };
  template<typename T>
  const _Vec3<T> _Vec3<T>::X    = { 1.0, 0.0, 0.0 };
  template<typename T>
  const _Vec3<T> _Vec3<T>::Y    = { 0.0, 1.0, 0.0 };
  template<typename T>
  const _Vec3<T> _Vec3<T>::Z    = { 0.0, 0.0, 1.0 };


  template<typename T>
  _Vec3<T> operator+(const _Vec3<T> &a, const _Vec3<T> &b) {
    _Vec3<T> r(a);
    r += b;
    return r;
  }


  template<typename T>
  _Vec3<T> &operator+=(_Vec3<T> &a, const _Vec3<T> &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
  }


  template<typename T>
  _Vec3<T> operator-(const _Vec3<T> &a, const _Vec3<T> &b) {
    _Vec3<T> r(a);
    r -= b;
    return r;
  }


  template<typename T>
  _Vec3<T> &operator-=(_Vec3<T> &a, const _Vec3<T> &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
  }


  template<typename T>
  _Vec3<T> operator-(const _Vec3<T> &a) {
    return { -a.x, -a.y, -a.z };
  }


  template<typename T>
  _Vec3<T> operator*(const T a, const _Vec3<T> &b) {
    _Vec3<T> r(b);
    r *= a;
    return r;
  }


  template<typename T>
  _Vec3<T> operator*(const _Vec3<T> &a, const T b) {
    return b * a;
  }


  template<typename T>
  _Vec3<T> &operator*=(_Vec3<T> &a, const T b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    return a;
  }


  template<typename T>
  _Vec3<T> operator/(const _Vec3<T> &a, const T b) {
    _Vec3<T> r(a);
    r /= b;
    return r;
  }


  template<typename T>
  _Vec3<T> &operator/=(_Vec3<T> &a, const T b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    return a;
  }


  // ========
  // = Vec4 =
  // ========


  template<typename T>
  struct _Vec4 {
    T x, y, z, w;


    T length_squared() const {
      return x * x + y * y + z * z + w * w;
    }


    T length() const {
      return std::sqrt(length_squared());
    }


    _Vec4<T> normalize() const {
      _Vec4<T> r(*this);
      r /= length();
      return r;
    }

    
    operator _Quat<T>() const {
      return _Quat<T>(x, y, z, w);
    }


    operator _Quat<T>&() {
      return *reinterpret_cast<_Quat<T>*>(this);
    }


    operator const _Quat<T>&() const {
      return *reinterpret_cast<const _Quat<T>*>(this);
    }


    static T dot(const _Vec4<T> &a, const _Vec4<T> &b) {
      return a.x * b.x + a.y * b.x + a.z * b.z + a.w * b.w;
    }


    static const _Vec4<T> ZERO;
    static const _Vec4<T> ONE;
    static const _Vec4<T> INF;
    static const _Vec4<T> X;
    static const _Vec4<T> Y;
    static const _Vec4<T> Z;
    static const _Vec4<T> W;
  };

  template<typename T>
  const _Vec4<T> _Vec4<T>::ZERO = { 0.0, 0.0, 0.0, 0.0 };
  template<typename T>
  const _Vec4<T> _Vec4<T>::ONE  = { 1.0, 1.0, 1.0, 1.0 };
  template<typename T>
  const _Vec4<T> _Vec4<T>::INF  = { std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity() };
  template<typename T>
  const _Vec4<T> _Vec4<T>::X    = { 1.0, 0.0, 0.0, 0.0 };
  template<typename T>
  const _Vec4<T> _Vec4<T>::Y    = { 0.0, 1.0, 0.0, 0.0 };
  template<typename T>
  const _Vec4<T> _Vec4<T>::Z    = { 0.0, 0.0, 1.0, 0.0 };
  template<typename T>
  const _Vec4<T> _Vec4<T>::W    = { 0.0, 0.0, 0.0, 1.0 };


  template<typename T>
  _Vec4<T> operator+(const _Vec4<T> &a, const _Vec4<T> &b) {
    _Vec4<T> r(a);
    r += b;
    return r;
  }


  template<typename T>
  _Vec4<T> &operator+=(_Vec4<T> &a, const _Vec4<T> &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
    return a;
  }


  template<typename T>
  _Vec4<T> operator-(const _Vec4<T> &a, const _Vec4<T> &b) {
    _Vec4<T> r(a);
    r -= b;
    return r;
  }


  template<typename T>
  _Vec4<T> &operator-=(_Vec4<T> &a, const _Vec4<T> &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
    return a;    
  }


  template<typename T>
  _Vec4<T> operator-(const _Vec4<T> &a) {
    return { -a.x, -a.y, -a.z, -a.w };
  }


  template<typename T>
  _Vec4<T> operator*(const T a, const _Vec4<T> &b) {
    _Vec4<T> r(b);
    r *= a;
    return r;
  }


  template<typename T>
  _Vec4<T> operator*(const _Vec4<T> &a, const T b) {
    return b * a;
  }


  template<typename T>
  _Vec4<T> &operator*=(_Vec4<T> &a, const T b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
    return a;
  }


  template<typename T>
  _Vec4<T> operator/(const _Vec4<T> &a, const T b) {
    _Vec4<T> r(a);
    r /= b;
    return r;
  }


  template<typename T>
  _Vec4<T> &operator/=(_Vec4<T> &a, const T b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
    return a;
  }


  // ==============
  // = Quaternion =
  // ==============
  

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


  // ====================
  // = Dual Quaternions =
  // ====================
  

  template<typename T>
  struct _DQuat {
    T yz, zx, xy;
    T w;
    T dx, dy, dz;
    T dxyz;


    _Quat<T> &real_part() {
      return *reinterpret_cast<_Quat<T>*>(this);
    }


    const _Quat<T> &real_part() const {
      return *reinterpret_cast<const _Quat<T>*>(this);
    }


    _Quat<T> &dual_part() {
      return *(1 + reinterpret_cast<_Quat<T>*>(this));
    }


    const _Quat<T> &dual_part() const {
      return *(1 + reinterpret_cast<const _Quat<T>*>(this));
    }

    
    // Returns the rotation quaternion linked with this (unit) dual quaternion
    _Quat<T> get_rotation() const {
      return real_part();
    }


    // Returns the translation component of this (unit) dual quaternion
    _Vec3<T> get_translation() const {
      const _Quat<T> &real = real_part();
      const _Quat<T> &dual = dual_part();
      _Quat<T> translation = ((T)2.0) * dual * real.conjugate();
      return _Vec3<T>(translation.x, translation.y, translation.z);
    }


    // If this dual quaternion represents a point, returns the coordinates of this point
    _Vec3<T> get_point() const {
      return _Vec3<T>(dx, dy, dz);
    }


    // Returns the screw coordinates (I, m, theta, d)
    void get_screw_coordinates(_Vec3<T> &direction, _Vec3<T> &moment, T &angle, T &translation) const {
      angle = 2.0 * std::acos(w);
      if (!is_approx_zero(angle)) {
        T inv_sin_a = std::sin(0.5 * angle);
        direction = inv_sin_a * (_Vec3<T>&)real_part();
        translation = -2.0 * w * inv_sin_a;
        moment = inv_sin_a * (_Vec3<T>&)dual_part() - ((T)0.5) * translation * w * inv_sin_a * direction;
      } else {
        direction = get_translation();
        translation = direction.length();
        if (!is_approx_zero(translation)) { // Normalize translation direction
          direction /= translation;
        } else {
          direction = _Vec3<T>::ZERO;
        }
        moment = _Vec3<T>::INF;
      }
    }


    _DQuat<T> dual_conjugate() const {
      return _DQuat<T>(
        yz, zx, xy, w,
        -dx, -dy, -dz, -dxyz
      );
    }


    _DQuat<T> quat_conjugate() const {
      return _DQuat<T>(
        -yz, -zx, -xy, w,
        -dx, -dy, -dz, dxyz
      );
    }

    _DQuat<T> dquat_conjugate() const {
      return _DQuat<T>(
        -yz, -zx, -xy, w,
        dx, dy, dz, -dxyz
      );
    }


    _DQuat<T> conjugate(const _DQuat<T> &q) const {
      return *this * q * inverse();
    }


    _DQuat<T> unit_conjugate(const _DQuat<T> &q) const {
      return *this * q * dquat_conjugate();
    }


    T length() const {
      return real_part().length();
    }


    _DQuat<T> inverse() {
      const _Quat<T> &real = real_part();
      const _Quat<T> &dual = dual_part();
      _Quat real_inv = inverse(real);
      return _DQuat(real_inv, -dual * real_inv);
    }
    

    // Tries to normalize this dual quaternion. Note that it may still not be a unit dual quaternion after this operation.
    _DQuat<T> &normalize() {
      T len = this->length();
      *this /= len;
      return *this;
    }


    static _DQuat<T> from_point(const _Vec3<T> &point) {
      return _DQuat<T>(0.0, 0.0, 0.0, 1.0, point.x, point.y, point.z, 0.0);
    }


    static _DQuat<T> from_line(const _Vec3<T> &point, const _Vec3<T> &direction) {
      T dir_mag2 = point.length_squared();
      // (l, m) is the Plücker coordinates of the line defined by (point, direction)
      _Vec3<T> l = point;
      if (!is_approx(dir_mag2, 1.0)) {
        l /= std::sqrt(dir_mag2);
      }

      _Vec3<T> m = cross(point, l);

      return _DQuat<T>( _Quat<T>(l, 0.0), _Quat<T>(m, 0.0) );
    }

    
    static _DQuat<T> from_axis_angle(const _Vec3<T> &axis, const T angle) {
      return _DQuat<T>(_Quat<T>::from_axis_angle(axis, angle), _Quat<T>::ZERO);
    }


    static _DQuat<T> from_translation(const _Vec3<T> &translation) {
      return _DQuat<T>(_Quat<T>::IDENTITY, _Quat<T>(((T)0.5) * translation, 0.0));
    }


    static _DQuat<T> from_axis_angle_translation(const _Vec3<T> &axis, const T angle, const _Vec3<T> &translation) {
      _Quat<T> rot = _Quat<T>::from_axis_angle(axis, angle);
      _Quat<T> trans = {((T)0.5) * translation, 0.0};
      return _DQuat<T>(rot, trans * rot);
    }


    static _DQuat<T> from_rotation(const _Quat<T> &rotation) {
      return _DQuat<T>(rotation, _Quat<T>::ZERO);
    }


    static _DQuat<T> from_rotation_translation(const _Quat<T> &rotation, const _Vec3<T> &translation) {
      _Quat<T> trans = {((T)0.5) * translation, 0.0};
      return _DQuat<T>(rotation, trans * rotation);
    }


    static _DQuat<T> from_screw_coordinates(const _Vec3<T> &direction, const _Vec3<T> &moment, const T &angle, const T &translation) {
      if (!is_approx_zero(angle)) {
        T cos_a = std::cos(angle / 2.0);
        T sin_a = std::sin(angle / 2.0);
        return _DQuat<T>(
          _Quat<T>(sin_a * direction, cos_a),
          _Quat<T>(sin_a * moment + (T)(0.5 * translation * cos_a) * direction, (T)-0.5 * translation * sin_a)
        );
      } else {
        return _DQuat<T>::from_translation(translation * direction);
      }
    }


    _DQuat(const T yz, const T zx, const T xy, const T w, const T dx, const T dy, const T dz, const T dxyz)
      : yz(yz), zx(zx), xy(xy), w(w), dx(dx), dy(dy), dz(dz), dxyz(dxyz) {}
    _DQuat(const _Quat<T> &real, const _Quat<T> &dual)
      : yz(real.x), zx(real.y), xy(real.z), w(real.w), dx(dual.x), dy(dual.y), dz(dual.z), dxyz(dual.w) {}

    static const _DQuat<T> IDENTITY;
  };

  template<typename T>
  const _DQuat<T> _DQuat<T>::IDENTITY = { _Quat<T>::IDENTITY, _Quat<T>::ZERO };


  template<typename T>
  _DQuat<T> seplerp(const _DQuat<T> &a, const _DQuat<T> &b, const T t) {
    _Vec3<T> a_trans = a.get_translation();
    _Vec3<T> b_trans = b.get_translation();
    _Quat<T> a_rot = a.get_rotation();
    _Quat<T> b_rot = b.get_rotation();
    return _DQuat<T>::from_rotation_translation(slerp(a_rot, b_rot, t), lerp(a_trans, b_trans, t));
  }


  template<typename T>
  _DQuat<T> sclerp(const _DQuat<T> &a, const _DQuat<T> &b, const T t) {
    _DQuat<T> delta = a.quat_conjugate() * b;
    // Get delta's screw coordinates
    _Vec3<T> direction, moment;
    T angle, translation;
    delta.get_screw_coordinates(direction, moment, angle, translation);
    // Calculate delta := delta^t - the delta to be used for the translation
    delta = _DQuat<T>::from_screw_coordinates(direction, moment, t * angle, t * translation);
    return a * delta;
  }


  template<typename T>
  _DQuat<T> kenlerp(const _DQuat<T> &a, const _DQuat<T> &b, const T t, const T beta) {
    _DQuat<T> sc_res = sclerp(a, b, t);
    _Quat<T> sc_rot = sc_res.get_rotation();
    _Vec3<T> sc_trans = sc_res.get_translation();
    _DQuat<T> sep_res = seplerp(a, b, t);
    _Quat<T> sep_rot = sep_res.get_rotation();
    _Vec3<T> sep_trans = sep_res.get_translation();
    return _DQuat<T>::from_rotation_translation(slerp(sc_rot, sep_rot, beta), lerp(sc_trans, sep_trans, beta));
  }


  template<typename T>
  _DQuat<T> operator+(const _DQuat<T> &a, const _DQuat<T> &b) {
    _DQuat<T> r(a);
    r += b;
    return r;
  }


  template<typename T>
  _DQuat<T> &operator+=(_DQuat<T> &a, const _DQuat<T> &b) {
    a.yz   += b.yz;
    a.zx   += b.zx;
    a.xy   += b.xy;
    a.w    += b.w;
    a.dx   += b.dx;
    a.dy   += b.dy;
    a.dz   += b.dz;
    a.dxyz += b.dxyz;
    return a;
  }


  template<typename T>
  _DQuat<T> operator-(const _DQuat<T> &a, const _DQuat<T> &b) {
    _DQuat<T> r(a);
    r -= b;
    return r;
  }


  template<typename T>
  _DQuat<T> &operator-=(_DQuat<T> &a, const _DQuat<T> &b) {
    a.yz   -= b.yz;
    a.zx   -= b.zx;
    a.xy   -= b.xy;
    a.w    -= b.w;
    a.dx   -= b.dx;
    a.dy   -= b.dy;
    a.dz   -= b.dz;
    a.dxyz -= b.dxyz;
    return a;
  }


  template<typename T>
  _DQuat<T> operator-(const _DQuat<T> &a) {
    return _DQuat<T>(
      -a.yz, -a.zx, -a.xy, -a.w,
      -a.dx, -a.dy, -a.dz, -a.dxyz
    );
  }


  template<typename T>
  _DQuat<T> operator*(const T a, const _DQuat<T> &b) {
    _DQuat<T> r(b);
    r *= a;
    return r;
  }


  template<typename T>
  _DQuat<T> operator*(const _DQuat<T> &b, const T a) {
    return a * b;
  }


  template<typename T>
  _DQuat<T> &operator*=(_DQuat<T> &a, const T b) {
    a.yz   *= b;
    a.zx   *= b;
    a.xy   *= b;
    a.w    *= b;
    a.dx   *= b;
    a.dy   *= b;
    a.dz   *= b;
    a.dxyz *= b;
    return a;
  }


  template<typename T>
  _DQuat<T> operator/(const _DQuat<T> &a, const T b) {
    _DQuat<T> r(a);
    r /= b;
    return r;
  }


  template<typename T>
  _DQuat<T> &operator/=(_DQuat<T> &a, const T b) {
    a.yz   /= b;
    a.zx   /= b;
    a.xy   /= b;
    a.w    /= b;
    a.dx   /= b;
    a.dy   /= b;
    a.dz   /= b;
    a.dxyz /= b;
    return a;
  }


  template<typename T>
  _DQuat<T> operator*(const _DQuat<T> &a, const _DQuat<T> &b) {
    return _DQuat<T>(
      a.w * b.yz + a.yz * b.w + a.zx * b.xy - a.xy * b.zx,
      a.w * b.zx + a.zx * b.w + a.xy * b.yz - a.yz * b.xy,
      a.w * b.xy + a.xy * b.w + a.yz * b.zx - a.zx * b.yz,
      -a.yz * b.yz - a.zx * b.zx - a.xy * b.xy + a.w * b.w,

      a.dxyz * b.yz + a.dx * b.w + a.dy * b.xy - a.dz * b.zx + a.w * b.dx + a.yz * b.dxyz + a.zx * b.dz - a.xy * b.dy,
      a.dxyz * b.zx + a.dy * b.w + a.dz * b.yz - a.dx * b.xy + a.w * b.dy + a.zx * b.dxyz + a.xy * b.dx - a.yz * b.dz,
      a.dxyz * b.xy + a.dz * b.w + a.dx * b.zx - a.dy * b.yz + a.w * b.dz + a.xy * b.dxyz + a.yz * b.dy - a.zx * b.dx,
      -a.dx * b.yz - a.dy * b.zx - a.dz * b.xy + a.dxyz * b.w + -a.yz * b.dx - a.zx * b.dy - a.xy * b.dz + a.w * b.dxyz
    );
  }


  template<typename T>
  _DQuat<T> &operator*=(_DQuat<T> &a, const _DQuat<T> &b) {
    a = a * b;
    return a;
  }


  // ========
  // = Mat3 =
  // ========
  

  template<typename T>
  struct _Mat3 {
    _Vec3<T> x, y, z;
  };


  template<typename T>
  _Vec3<T> operator*(const _Mat3<T> &a, const _Vec3<T> &b) {
    return _Vec3<T>(
      a.x.x * b.x + a.y.x * b.y + a.z.x * b.z,
      a.x.y * b.x + a.y.y * b.y + a.z.y * b.z,
      a.x.z * b.x + a.y.z * b.y + a.z.z * b.z
    );
  }


  template<typename T>
  _Vec3<T> operator*(const _Vec3<T> &a, const _Mat3<T> &b) {
    return _Vec3<T>(
      _Vec3<T>::dot(a, b.x),
      _Vec3<T>::dot(a, b.y),
      _Vec3<T>::dot(a, b.z)
    );
  }


  template<typename T>
  _Mat3<T> operator*(const _Mat3<T> &a, const _Mat3<T> &b) {
    return _Mat3<T>(
      _Vec3<T>(a * b.x),
      _Vec3<T>(a * b.y),
      _Vec3<T>(a * b.z)
    );
  }


  template<typename T>
  _Mat3<T> &operator*=(_Mat3<T> &a, const _Mat3<T> &b) {
    a = a * b;
    return a;
  }


  // ========
  // = Mat4 =
  // ========

  template<typename T>
  struct _Mat4 {
    _Vec4<T> x, y, z, w;
  };



  template<typename T>
  _Vec4<T> operator*(const _Mat4<T> &a, const _Vec4<T> &b) {
    return _Vec4(
      a.x.x * b.x + a.y.x * b.y + a.z.x * b.z + a.w.x * b.w,
      a.x.y * b.x + a.y.y * b.y + a.z.y * b.z + a.w.y * b.w,
      a.x.z * b.x + a.y.z * b.y + a.z.z * b.z + a.w.z * b.w,
      a.x.w * b.x + a.y.w * b.y + a.z.w * b.z + a.w.w * b.w
    );
  }


  template<typename T>
  _Vec4<T> operator*(const _Vec4<T> &a, const _Mat4<T> &b) {
    return _Vec4(
      _Vec4<T>::dot(a, b.x),
      _Vec4<T>::dot(a, b.y),
      _Vec4<T>::dot(a, b.z),
      _Vec4<T>::dot(a, b.w)
    );
  }


  template<typename T>
  _Mat4<T> operator*(const _Mat4<T> &a, const _Mat4<T> &b) {
    return _Mat4<T>(
      _Vec4<T>(a * b.x),
      _Vec4<T>(a * b.y),
      _Vec4<T>(a * b.z),
      _Vec4<T>(a * b.w)
    );
  }


  template<typename T>
  _Mat4<T> &operator*=(_Mat4<T> &a, const _Mat4<T> &b) {
    a = a * b;
    return a;
  }


  // ===================
  // = Print operators =
  // ===================
  

  template<typename T>
  std::ostream &operator<<(std::ostream &os, const _Vec3<T> &v) {
    os << "Vec3( " << v.x << ", " << v.y << ", " << v.z << " )";
    return os;
  }


  template<typename T>
  std::ostream &operator<<(std::ostream &os, const _Vec4<T> &v) {
    os << "Vec4( " << v.x << ", " << v.y << ", " << v.z << ", " << v.w << " )";
    return os;
  }


  template<typename T>
  std::ostream &operator<<(std::ostream &os, const _Quat<T> &q) {
    os << q.x << " i + " << q.y << " j + " << q.z << " k + " << q.w;
    return os;
  }


  template<typename T>
  std::ostream &operator<<(std::ostream &os, const _DQuat<T> &q) {
    os << q.yz << " i + " << q.zx << " j + " << q.xy << " k + " << q.w << " + " << q.dx << " εi + " << q.dy << " εj + " << q.dz << " εk + " << q.dxyz << " ε";
    return os;
  }
}
