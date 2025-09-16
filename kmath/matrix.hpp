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


namespace kmath {
  

  // ========
  // = Mat2 =
  // ========


  template<typename T>
  struct _Mat2 {
    _Vec2<T> x, y;
  };


  template<typename T>
  KMATH_FUNC _Mat2<T> transpose(const _Mat2<T> &m) {
    return _Mat2<T>(
      _Vec2<T>(m.x.x, m.y.x),
      _Vec2<T>(m.x.y, m.y.y)
    );
  }

  
  template<typename T>
  KMATH_FUNC _Vec2<T> operator*(const _Mat2<T> &a, const _Vec2<T> &b) {
    return _Vec2<T>(
      a.x.x * b.x + a.y.x * b.y,
      a.x.y * b.x + a.y.x * b.y
    );
  }


  template<typename T>
  KMATH_FUNC _Vec2<T> operator*(const _Vec2<T> &a, const _Mat2<T> &b) {
    return _Vec2<T>(
      kmath::dot(a.x, b),
      kmath::dot(a.y, b)
    );
  }


  template<typename T>
  KMATH_FUNC _Mat2<T> operator*(const _Mat2<T> &a, const _Mat2<T> &b) {
    return _Mat2<T>(
      a * b.x,
      a * b.y
    );
  }


  template<typename T>
  KMATH_FUNC _Mat2<T> &operator*=(_Mat2<T> &a, const _Mat2<T> &b) {
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
  KMATH_FUNC _Mat3<T> transpose(const _Mat3<T> &m) {
    return _Mat3<T>(
      _Vec3<T>(m.x.x, m.y.x, m.z.x),
      _Vec3<T>(m.x.y, m.y.y, m.z.y),
      _Vec3<T>(m.x.z, m.y.z, m.z.z)
    );
  }
  

  template<typename T>
  KMATH_FUNC _Vec3<T> operator*(const _Mat3<T> &a, const _Vec3<T> &b) {
    return _Vec3<T>(
      a.x.x * b.x + a.y.x * b.y + a.z.x * b.z,
      a.x.y * b.x + a.y.y * b.y + a.z.y * b.z,
      a.x.z * b.x + a.y.z * b.y + a.z.z * b.z
    );
  }


  template<typename T>
  KMATH_FUNC _Vec3<T> operator*(const _Vec3<T> &a, const _Mat3<T> &b) {
    return _Vec3<T>(
      kmath::dot(a, b.x),
      kmath::dot(a, b.y),
      kmath::dot(a, b.z)
    );
  }


  template<typename T>
  KMATH_FUNC _Mat3<T> operator*(const _Mat3<T> &a, const _Mat3<T> &b) {
    return _Mat3<T>(
      _Vec3<T>(a * b.x),
      _Vec3<T>(a * b.y),
      _Vec3<T>(a * b.z)
    );
  }


  template<typename T>
  KMATH_FUNC _Mat3<T> &operator*=(_Mat3<T> &a, const _Mat3<T> &b) {
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
  KMATH_FUNC _Mat4<T> transpose(const _Mat4<T> &m) {
    return _Mat4<T>(
      _Vec4<T>(m.x.x, m.y.x, m.z.x, m.w.x),
      _Vec4<T>(m.x.y, m.y.y, m.z.y, m.w.y),
      _Vec4<T>(m.x.z, m.y.z, m.z.z, m.w.z),
      _Vec4<T>(m.x.w, m.y.w, m.z.w, m.w.w)
    );
  }


  template<typename T>
  KMATH_FUNC _Vec4<T> operator*(const _Mat4<T> &a, const _Vec4<T> &b) {
    return _Vec4(
      a.x.x * b.x + a.y.x * b.y + a.z.x * b.z + a.w.x * b.w,
      a.x.y * b.x + a.y.y * b.y + a.z.y * b.z + a.w.y * b.w,
      a.x.z * b.x + a.y.z * b.y + a.z.z * b.z + a.w.z * b.w,
      a.x.w * b.x + a.y.w * b.y + a.z.w * b.z + a.w.w * b.w
    );
  }


  template<typename T>
  KMATH_FUNC _Vec4<T> operator*(const _Vec4<T> &a, const _Mat4<T> &b) {
    return _Vec4(
      kmath::dot(a, b.x),
      kmath::dot(a, b.y),
      kmath::dot(a, b.z),
      kmath::dot(a, b.w)
    );
  }


  template<typename T>
  KMATH_FUNC _Mat4<T> operator*(const _Mat4<T> &a, const _Mat4<T> &b) {
    return _Mat4<T>(
      _Vec4<T>(a * b.x),
      _Vec4<T>(a * b.y),
      _Vec4<T>(a * b.z),
      _Vec4<T>(a * b.w)
    );
  }


  template<typename T>
  KMATH_FUNC _Mat4<T> &operator*=(_Mat4<T> &a, const _Mat4<T> &b) {
    a = a * b;
    return a;
  }


  // ================
  // = Type aliases =
  // ================


  typedef _Mat2<float> Mat2;
  typedef _Mat2<double> Mat2d;

  typedef _Mat3<float> Mat3;
  typedef _Mat3<double> Mat3d;

  typedef _Mat4<float> Mat4;
  typedef _Mat4<double> Mat4d;
  

}
