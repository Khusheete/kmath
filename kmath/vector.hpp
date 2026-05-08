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

#include <limits>


namespace kmath {


  // ===========
  // = Vector2 =
  // ===========
  

  template<Number T>
  struct _Vec2 {
    T x, y;

  public:
    constexpr _Vec2(): _Vec2(ZERO) {}
    constexpr _Vec2(const T v): x(v), y(v) {}
    constexpr _Vec2(const T x, const T y): x(x), y(y) {}

    template<Castable<T> Q>
    constexpr explicit _Vec2(const _Vec2<Q> &v): x(Q(v.x)), y(Q(v.y)) {}

  public:
    constexpr _Vec2<T> xx() const { return _Vec2<T>(x, x); }
    constexpr _Vec2<T> yy() const { return _Vec2<T>(y, y); }
    constexpr _Vec2<T> xy() const { return _Vec2<T>(x, y); }
    constexpr _Vec2<T> yx() const { return _Vec2<T>(y, x); }

  public:
    constexpr T &operator[](const size_t index) { return reinterpret_cast<T*>(this)[index]; }
    constexpr const T &operator[](const size_t index) const { return reinterpret_cast<const T*>(this)[index]; }

  public:
    static constexpr const size_t SIZE = 2;

  public:
    static const _Vec2<T> ZERO;
    static const _Vec2<T> ONE;
    static const _Vec2<T> INF;
    static const _Vec2<T> X;
    static const _Vec2<T> Y;
  };


  template<Number T>
  constexpr const _Vec2<T> _Vec2<T>::ZERO = _Vec2<T>(T(0), T(0));
  template<Number T>
  constexpr const _Vec2<T> _Vec2<T>::ONE = _Vec2<T>(T(1), T(1));
  template<Number T>
  constexpr const _Vec2<T> _Vec2<T>::INF = _Vec2<T>(std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity());
  template<Number T>
  constexpr const _Vec2<T> _Vec2<T>::X = _Vec2<T>(T(1), T(0));
  template<Number T>
  constexpr const _Vec2<T> _Vec2<T>::Y = _Vec2<T>(T(0), T(1));


  // =====================
  // = Vector2 Functions =
  // =====================


  template<Number T>
  constexpr T length_squared(const _Vec2<T> &v) {
    return v.x * v.x + v.y * v.y;
  }
  

  template<Number T>
  constexpr T length(const _Vec2<T> &v) {
    return sqrt(length_squared(v));
  }


  template<Number T>
  constexpr T distance_squared(const _Vec2<T> &a, const _Vec2<T> &b) {
    return length_squared(b - a);
  }


  template<Number T>
  constexpr T distance(const _Vec2<T> &a, const _Vec2<T> &b) {
    return length(b - a);
  }
  

  template<Number T>
  constexpr _Vec2<T> normalized(const _Vec2<T> &v) {
    return v / length(v);
  }
    

  template<Number T>
  constexpr T dot(const _Vec2<T> &a, const _Vec2<T> &b) {
    return a.x * b.x + a.y * b.y;
  }


  // =====================
  // = Vector2 Operators =
  // =====================


  template<Number T>
  constexpr _Vec2<T> operator+(const _Vec2<T> &a, const _Vec2<T> &b) {
    _Vec2<T> res(a);
    res += b;
    return res;
  }


  template<Number T>
  constexpr _Vec2<T> &operator+=(_Vec2<T> &a, const _Vec2<T> &b) {
    a.x += b.x;
    a.y += b.y;
    return a;
  }


  template<Number T>
  constexpr _Vec2<T> operator-(const _Vec2<T> &a, const _Vec2<T> &b) {
    _Vec2<T> res(a);
    res -= b;
    return res;
  }


  template<Number T>
  constexpr _Vec2<T> &operator-=(_Vec2<T> &a, const _Vec2<T> &b) {
    a.x -= b.x;
    a.y -= b.y;
    return a;
  }


  template<Number T>
  constexpr _Vec2<T> operator-(const _Vec2<T> &a) {
    return _Vec2<T>(
      -a.x, -a.y
    );
  }


  template<Number T>
  constexpr _Vec2<T> operator*(const _Vec2<T> &a, const _Vec2<T> &b) {
    _Vec2<T> res(a);
    res *= b;
    return res;
  }


  template<Number T>
  constexpr _Vec2<T> &operator*=(_Vec2<T> &a, const _Vec2<T> &b) {
    a.x *= b.x;
    a.y *= b.y;
    return a;
  }


  template<Number T>
  constexpr _Vec2<T> operator/(const _Vec2<T> &a, const _Vec2<T> &b) {
    _Vec2<T> res(a);
    res /= b;
    return res;
  }


  template<Number T>
  constexpr _Vec2<T> &operator/=(_Vec2<T> &a, const _Vec2<T> &b) {
    a.x /= b.x;
    a.y /= b.y;
    return a;
  }


  template<Number T>
  constexpr _Vec2<T> operator*(const T s, const _Vec2<T> &v) {
    return _Vec2<T>(
      s * v.x,
      s * v.y
    );
  }


  template<Number T>
  constexpr _Vec2<T> operator*(const _Vec2<T> &v, const T s) {
    return _Vec2<T>(
      v.x * s,
      v.y * s    
    );
  }


  template<Number T>
  constexpr _Vec2<T> &operator*=(_Vec2<T> &v, const T s) {
    v = v * s;
    return v;
  }


  template<Number T>
  constexpr _Vec2<T> operator/(const _Vec2<T> &v, const T s) {
    _Vec2<T> res(v);
    res /= s;
    return res;
  }


  template<Number T>
  constexpr _Vec2<T> &operator/=(_Vec2<T> &v, const T s) {
    const T scale = T(1) / s;
    return v *= scale;
  }


  template<Number T>
  constexpr bool operator==(const _Vec2<T> &a, const _Vec2<T> &b) {
    return a.x == b.x && a.y == b.y;
  }


  template<Number T>
  constexpr bool operator!=(const _Vec2<T> &a, const _Vec2<T> &b) {
    return !(a == b);
  }


  // ===========
  // = Vector3 =
  // ===========
  

  template<Number T>
  struct _Vec3 {
    T x, y, z;

  public:
    constexpr _Vec3(): _Vec3(ZERO) {}
    constexpr _Vec3(const T v): x(v), y(v), z(v) {}
    constexpr _Vec3(const T x, const T y, const T z): x(x), y(y), z(z) {}
    constexpr _Vec3(const _Vec2<T> &a, const T b): x(a.x), y(a.y), z(b) {}
    constexpr _Vec3(const T a, const _Vec2<T> &b): x(a), y(b.x), z(b.y) {}

    template<Castable<T> Q>
    constexpr explicit _Vec3(const _Vec3<Q> &v): x(Q(v.x)), y(Q(v.y)), z(Q(v.z)) {}

  public:
    constexpr _Vec2<T> xx() const { return _Vec2<T>(x, x); }
    constexpr _Vec2<T> yy() const { return _Vec2<T>(y, y); }
    constexpr _Vec2<T> zz() const { return _Vec2<T>(z, z); }
    constexpr _Vec2<T> xy() const { return _Vec2<T>(x, y); }
    constexpr _Vec2<T> yx() const { return _Vec2<T>(y, x); }
    constexpr _Vec2<T> zy() const { return _Vec2<T>(z, y); }
    constexpr _Vec2<T> yz() const { return _Vec2<T>(y, z); }
    constexpr _Vec2<T> xz() const { return _Vec2<T>(x, z); }
    constexpr _Vec2<T> zx() const { return _Vec2<T>(z, x); }

    constexpr _Vec3<T> xxx() const { return _Vec3<T>(x, x, x); }
    constexpr _Vec3<T> xyy() const { return _Vec3<T>(x, y, y); }
    constexpr _Vec3<T> xzz() const { return _Vec3<T>(x, z, z); }
    constexpr _Vec3<T> xxy() const { return _Vec3<T>(x, x, y); }
    constexpr _Vec3<T> xyx() const { return _Vec3<T>(x, y, x); }
    constexpr _Vec3<T> xzy() const { return _Vec3<T>(x, z, y); }
    constexpr _Vec3<T> xyz() const { return _Vec3<T>(x, y, z); }
    constexpr _Vec3<T> xxz() const { return _Vec3<T>(x, x, z); }
    constexpr _Vec3<T> xzx() const { return _Vec3<T>(x, z, x); }
    constexpr _Vec3<T> yxx() const { return _Vec3<T>(y, x, x); }
    constexpr _Vec3<T> yyy() const { return _Vec3<T>(y, y, y); }
    constexpr _Vec3<T> yzz() const { return _Vec3<T>(y, z, z); }
    constexpr _Vec3<T> yxy() const { return _Vec3<T>(y, x, y); }
    constexpr _Vec3<T> yyx() const { return _Vec3<T>(y, y, x); }
    constexpr _Vec3<T> yzy() const { return _Vec3<T>(y, z, y); }
    constexpr _Vec3<T> yyz() const { return _Vec3<T>(y, y, z); }
    constexpr _Vec3<T> yxz() const { return _Vec3<T>(y, x, z); }
    constexpr _Vec3<T> yzx() const { return _Vec3<T>(y, z, x); }
    constexpr _Vec3<T> zxx() const { return _Vec3<T>(z, x, x); }
    constexpr _Vec3<T> zyy() const { return _Vec3<T>(z, y, y); }
    constexpr _Vec3<T> zzz() const { return _Vec3<T>(z, z, z); }
    constexpr _Vec3<T> zxy() const { return _Vec3<T>(z, x, y); }
    constexpr _Vec3<T> zyx() const { return _Vec3<T>(z, y, x); }
    constexpr _Vec3<T> zzy() const { return _Vec3<T>(z, z, y); }
    constexpr _Vec3<T> zyz() const { return _Vec3<T>(z, y, z); }
    constexpr _Vec3<T> zxz() const { return _Vec3<T>(z, x, z); }
    constexpr _Vec3<T> zzx() const { return _Vec3<T>(z, z, x); }

  public:
    constexpr T &operator[](const size_t index) { return reinterpret_cast<T*>(this)[index]; }
    constexpr const T &operator[](const size_t index) const { return reinterpret_cast<const T*>(this)[index]; }

  public:
    static constexpr const size_t SIZE = 3;

  public:
    static const _Vec3<T> ZERO;
    static const _Vec3<T> ONE;
    static const _Vec3<T> INF;
    static const _Vec3<T> X;
    static const _Vec3<T> Y;
    static const _Vec3<T> Z;
  };


  template<Number T>
  constexpr const _Vec3<T> _Vec3<T>::ZERO = _Vec3<T>(T(0), T(0), T(0));
  template<Number T>
  constexpr const _Vec3<T> _Vec3<T>::ONE = _Vec3<T>(T(1), T(1), T(1));
  template<Number T>
  constexpr const _Vec3<T> _Vec3<T>::INF = _Vec3<T>(std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity());
  template<Number T>
  constexpr const _Vec3<T> _Vec3<T>::X = _Vec3<T>(T(1), T(0), T(0));
  template<Number T>
  constexpr const _Vec3<T> _Vec3<T>::Y = _Vec3<T>(T(0), T(1), T(0));
  template<Number T>
  constexpr const _Vec3<T> _Vec3<T>::Z = _Vec3<T>(T(0), T(0), T(1));


  // =====================
  // = Vector3 Functions =
  // =====================


  template<Number T>
  constexpr T length_squared(const _Vec3<T> &v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
  }
  

  template<Number T>
  constexpr T length(const _Vec3<T> &v) {
    return sqrt(length_squared(v));
  }


  template<Number T>
  constexpr T distance_squared(const _Vec3<T> &a, const _Vec3<T> &b) {
    return length_squared(b - a);
  }


  template<Number T>
  constexpr T distance(const _Vec3<T> &a, const _Vec3<T> &b) {
    return length(b - a);
  }
  

  template<Number T>
  constexpr _Vec3<T> normalized(const _Vec3<T> &v) {
    return v / length(v);
  }
    

  template<Number T>
  constexpr T dot(const _Vec3<T> &a, const _Vec3<T> &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }


  template<Number T>
  constexpr _Vec3<T> cross(const _Vec3<T> &a, const _Vec3<T> &b) {
    return _Vec3<T>(
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x    
    );
  }


  template<Number T>
  constexpr _Vec2<T> homogeneous_projection(const _Vec3<T> &v) {
    return _Vec2<T>(v.x, v.y, v.z) / v.w;
  }
  

  // =====================
  // = Vector3 Operators =
  // =====================


  template<Number T>
  constexpr _Vec3<T> operator+(const _Vec3<T> &a, const _Vec3<T> &b) {
    _Vec3<T> res(a);
    res += b;
    return res;
  }


  template<Number T>
  constexpr _Vec3<T> &operator+=(_Vec3<T> &a, const _Vec3<T> &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
  }


  template<Number T>
  constexpr _Vec3<T> operator-(const _Vec3<T> &a, const _Vec3<T> &b) {
    _Vec3<T> res(a);
    res -= b;
    return res;
  }


  template<Number T>
  constexpr _Vec3<T> &operator-=(_Vec3<T> &a, const _Vec3<T> &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
  }


  template<Number T>
  constexpr _Vec3<T> operator-(const _Vec3<T> &a) {
    return _Vec3<T>(
      -a.x, -a.y, -a.z    
    );
  }


  template<Number T>
  constexpr _Vec3<T> operator*(const _Vec3<T> &a, const _Vec3<T> &b) {
    _Vec3<T> res(a);
    res *= b;
    return res;
  }


  template<Number T>
  constexpr _Vec3<T> &operator*=(_Vec3<T> &a, const _Vec3<T> &b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    return a;
  }


  template<Number T>
  constexpr _Vec3<T> operator/(const _Vec3<T> &a, const _Vec3<T> &b) {
    _Vec3<T> res(a);
    res /= b;
    return res;
  }


  template<Number T>
  constexpr _Vec3<T> &operator/=(_Vec3<T> &a, const _Vec3<T> &b) {
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
    return a;
  }


  template<Number T>
  constexpr _Vec3<T> operator*(const T s, const _Vec3<T> &v) {
    return _Vec3<T>(
      s * v.x,
      s * v.y,
      s * v.z    
    );
  }


  template<Number T>
  constexpr _Vec3<T> operator*(const _Vec3<T> &v, const T s) {
    return _Vec3<T>(
      v.x * s,
      v.y * s,
      v.z * s    
    );
  }


  template<Number T>
  constexpr _Vec3<T> &operator*=(_Vec3<T> &v, const T s) {
    v = v * s;
    return v;
  }


  template<Number T>
  constexpr _Vec3<T> operator/(const _Vec3<T> &v, const T s) {
    _Vec3<T> res(v);
    res /= s;
    return res;
  }


  template<Number T>
  constexpr _Vec3<T> &operator/=(_Vec3<T> &v, const T s) {
    const T scale = T(1) / s;
    return v *= scale;
  }


  template<Number T>
  constexpr bool operator==(const _Vec3<T> &a, const _Vec3<T> &b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }


  template<Number T>
  constexpr bool operator!=(const _Vec3<T> &a, const _Vec3<T> &b) {
    return !(a == b);
  }


  // ===========
  // = Vector4 =
  // ===========


  template<Number T>
  struct _Vec4 {
    T x, y, z, w;

  public:
    constexpr _Vec4(): _Vec4(ZERO) {}
    constexpr _Vec4(const T v): x(v), y(v), z(v), w(v) {}
    constexpr _Vec4(const T x, const T y, const T z, const T w): x(x), y(y), z(z), w(w) {}
    constexpr _Vec4(const _Vec2<T> &a, const T b, const T c): x(a.x), y(a.y), z(b), w(c) {}
    constexpr _Vec4(const T a, const _Vec2<T> &b, const T c): x(a), y(b.x), z(b.z), w(c) {}
    constexpr _Vec4(const T a, const T b, const _Vec2<T> &c): x(a), y(b), z(c.x), w(c.y) {}
    constexpr _Vec4(const _Vec2<T> &a, const _Vec2<T> &b): x(a.x), y(a.y), z(b.x), w(b.y) {}
    constexpr _Vec4(const _Vec3<T> &a, const T b): x(a.x), y(a.y), z(a.z), w(b) {}
    constexpr _Vec4(const T a, const _Vec3<T> &b): x(a), y(b.x), z(b.y), w(b.z) {}

    template<Castable<T> Q>
    constexpr explicit _Vec4(const _Vec4<Q> &v): x(Q(v.x)), y(Q(v.y)), z(Q(v.z)), w(Q(v.w)) {}

  public:
    constexpr _Vec2<T> xx() const { return _Vec2<T>(x, x); }
    constexpr _Vec2<T> yy() const { return _Vec2<T>(y, y); }
    constexpr _Vec2<T> zz() const { return _Vec2<T>(z, z); }
    constexpr _Vec2<T> ww() const { return _Vec2<T>(w, w); }
    constexpr _Vec2<T> xy() const { return _Vec2<T>(x, y); }
    constexpr _Vec2<T> yx() const { return _Vec2<T>(y, x); }
    constexpr _Vec2<T> zy() const { return _Vec2<T>(z, y); }
    constexpr _Vec2<T> yz() const { return _Vec2<T>(y, z); }
    constexpr _Vec2<T> xz() const { return _Vec2<T>(x, z); }
    constexpr _Vec2<T> zx() const { return _Vec2<T>(z, x); }
    constexpr _Vec2<T> xw() const { return _Vec2<T>(x, w); }
    constexpr _Vec2<T> wx() const { return _Vec2<T>(w, x); }
    constexpr _Vec2<T> yw() const { return _Vec2<T>(y, w); }
    constexpr _Vec2<T> wy() const { return _Vec2<T>(w, y); }
    constexpr _Vec2<T> zw() const { return _Vec2<T>(z, w); }
    constexpr _Vec2<T> wz() const { return _Vec2<T>(w, z); }

    constexpr _Vec3<T> xxx() const { return _Vec3<T>(x, x, x); }
    constexpr _Vec3<T> xyy() const { return _Vec3<T>(x, y, y); }
    constexpr _Vec3<T> xzz() const { return _Vec3<T>(x, z, z); }
    constexpr _Vec3<T> xww() const { return _Vec3<T>(x, w, w); }
    constexpr _Vec3<T> xxy() const { return _Vec3<T>(x, x, y); }
    constexpr _Vec3<T> xyx() const { return _Vec3<T>(x, y, x); }
    constexpr _Vec3<T> xzy() const { return _Vec3<T>(x, z, y); }
    constexpr _Vec3<T> xyz() const { return _Vec3<T>(x, y, z); }
    constexpr _Vec3<T> xxz() const { return _Vec3<T>(x, x, z); }
    constexpr _Vec3<T> xzx() const { return _Vec3<T>(x, z, x); }
    constexpr _Vec3<T> xxw() const { return _Vec3<T>(x, x, w); }
    constexpr _Vec3<T> xwx() const { return _Vec3<T>(x, w, x); }
    constexpr _Vec3<T> xyw() const { return _Vec3<T>(x, y, w); }
    constexpr _Vec3<T> xwy() const { return _Vec3<T>(x, w, y); }
    constexpr _Vec3<T> xzw() const { return _Vec3<T>(x, z, w); }
    constexpr _Vec3<T> xwz() const { return _Vec3<T>(x, w, z); }
    constexpr _Vec3<T> yxx() const { return _Vec3<T>(y, x, x); }
    constexpr _Vec3<T> yyy() const { return _Vec3<T>(y, y, y); }
    constexpr _Vec3<T> yzz() const { return _Vec3<T>(y, z, z); }
    constexpr _Vec3<T> yww() const { return _Vec3<T>(y, w, w); }
    constexpr _Vec3<T> yxy() const { return _Vec3<T>(y, x, y); }
    constexpr _Vec3<T> yyx() const { return _Vec3<T>(y, y, x); }
    constexpr _Vec3<T> yzy() const { return _Vec3<T>(y, z, y); }
    constexpr _Vec3<T> yyz() const { return _Vec3<T>(y, y, z); }
    constexpr _Vec3<T> yxz() const { return _Vec3<T>(y, x, z); }
    constexpr _Vec3<T> yzx() const { return _Vec3<T>(y, z, x); }
    constexpr _Vec3<T> yxw() const { return _Vec3<T>(y, x, w); }
    constexpr _Vec3<T> ywx() const { return _Vec3<T>(y, w, x); }
    constexpr _Vec3<T> yyw() const { return _Vec3<T>(y, y, w); }
    constexpr _Vec3<T> ywy() const { return _Vec3<T>(y, w, y); }
    constexpr _Vec3<T> yzw() const { return _Vec3<T>(y, z, w); }
    constexpr _Vec3<T> ywz() const { return _Vec3<T>(y, w, z); }
    constexpr _Vec3<T> zxx() const { return _Vec3<T>(z, x, x); }
    constexpr _Vec3<T> zyy() const { return _Vec3<T>(z, y, y); }
    constexpr _Vec3<T> zzz() const { return _Vec3<T>(z, z, z); }
    constexpr _Vec3<T> zww() const { return _Vec3<T>(z, w, w); }
    constexpr _Vec3<T> zxy() const { return _Vec3<T>(z, x, y); }
    constexpr _Vec3<T> zyx() const { return _Vec3<T>(z, y, x); }
    constexpr _Vec3<T> zzy() const { return _Vec3<T>(z, z, y); }
    constexpr _Vec3<T> zyz() const { return _Vec3<T>(z, y, z); }
    constexpr _Vec3<T> zxz() const { return _Vec3<T>(z, x, z); }
    constexpr _Vec3<T> zzx() const { return _Vec3<T>(z, z, x); }
    constexpr _Vec3<T> zxw() const { return _Vec3<T>(z, x, w); }
    constexpr _Vec3<T> zwx() const { return _Vec3<T>(z, w, x); }
    constexpr _Vec3<T> zyw() const { return _Vec3<T>(z, y, w); }
    constexpr _Vec3<T> zwy() const { return _Vec3<T>(z, w, y); }
    constexpr _Vec3<T> zzw() const { return _Vec3<T>(z, z, w); }
    constexpr _Vec3<T> zwz() const { return _Vec3<T>(z, w, z); }
    constexpr _Vec3<T> wxx() const { return _Vec3<T>(w, x, x); }
    constexpr _Vec3<T> wyy() const { return _Vec3<T>(w, y, y); }
    constexpr _Vec3<T> wzz() const { return _Vec3<T>(w, z, z); }
    constexpr _Vec3<T> www() const { return _Vec3<T>(w, w, w); }
    constexpr _Vec3<T> wxy() const { return _Vec3<T>(w, x, y); }
    constexpr _Vec3<T> wyx() const { return _Vec3<T>(w, y, x); }
    constexpr _Vec3<T> wzy() const { return _Vec3<T>(w, z, y); }
    constexpr _Vec3<T> wyz() const { return _Vec3<T>(w, y, z); }
    constexpr _Vec3<T> wxz() const { return _Vec3<T>(w, x, z); }
    constexpr _Vec3<T> wzx() const { return _Vec3<T>(w, z, x); }
    constexpr _Vec3<T> wxw() const { return _Vec3<T>(w, x, w); }
    constexpr _Vec3<T> wwx() const { return _Vec3<T>(w, w, x); }
    constexpr _Vec3<T> wyw() const { return _Vec3<T>(w, y, w); }
    constexpr _Vec3<T> wwy() const { return _Vec3<T>(w, w, y); }
    constexpr _Vec3<T> wzw() const { return _Vec3<T>(w, z, w); }
    constexpr _Vec3<T> wwz() const { return _Vec3<T>(w, w, z); }

    constexpr _Vec4<T> xxxx() const { return _Vec4<T>(x, x, x, x); }
    constexpr _Vec4<T> xxyy() const { return _Vec4<T>(x, x, y, y); }
    constexpr _Vec4<T> xxzz() const { return _Vec4<T>(x, x, z, z); }
    constexpr _Vec4<T> xxww() const { return _Vec4<T>(x, x, w, w); }
    constexpr _Vec4<T> xxxy() const { return _Vec4<T>(x, x, x, y); }
    constexpr _Vec4<T> xxyx() const { return _Vec4<T>(x, x, y, x); }
    constexpr _Vec4<T> xxzy() const { return _Vec4<T>(x, x, z, y); }
    constexpr _Vec4<T> xxyz() const { return _Vec4<T>(x, x, y, z); }
    constexpr _Vec4<T> xxxz() const { return _Vec4<T>(x, x, x, z); }
    constexpr _Vec4<T> xxzx() const { return _Vec4<T>(x, x, z, x); }
    constexpr _Vec4<T> xxxw() const { return _Vec4<T>(x, x, x, w); }
    constexpr _Vec4<T> xxwx() const { return _Vec4<T>(x, x, w, x); }
    constexpr _Vec4<T> xxyw() const { return _Vec4<T>(x, x, y, w); }
    constexpr _Vec4<T> xxwy() const { return _Vec4<T>(x, x, w, y); }
    constexpr _Vec4<T> xxzw() const { return _Vec4<T>(x, x, z, w); }
    constexpr _Vec4<T> xxwz() const { return _Vec4<T>(x, x, w, z); }
    constexpr _Vec4<T> xyxx() const { return _Vec4<T>(x, y, x, x); }
    constexpr _Vec4<T> xyyy() const { return _Vec4<T>(x, y, y, y); }
    constexpr _Vec4<T> xyzz() const { return _Vec4<T>(x, y, z, z); }
    constexpr _Vec4<T> xyww() const { return _Vec4<T>(x, y, w, w); }
    constexpr _Vec4<T> xyxy() const { return _Vec4<T>(x, y, x, y); }
    constexpr _Vec4<T> xyyx() const { return _Vec4<T>(x, y, y, x); }
    constexpr _Vec4<T> xyzy() const { return _Vec4<T>(x, y, z, y); }
    constexpr _Vec4<T> xyyz() const { return _Vec4<T>(x, y, y, z); }
    constexpr _Vec4<T> xyxz() const { return _Vec4<T>(x, y, x, z); }
    constexpr _Vec4<T> xyzx() const { return _Vec4<T>(x, y, z, x); }
    constexpr _Vec4<T> xyxw() const { return _Vec4<T>(x, y, x, w); }
    constexpr _Vec4<T> xywx() const { return _Vec4<T>(x, y, w, x); }
    constexpr _Vec4<T> xyyw() const { return _Vec4<T>(x, y, y, w); }
    constexpr _Vec4<T> xywy() const { return _Vec4<T>(x, y, w, y); }
    constexpr _Vec4<T> xyzw() const { return _Vec4<T>(x, y, z, w); }
    constexpr _Vec4<T> xywz() const { return _Vec4<T>(x, y, w, z); }
    constexpr _Vec4<T> xzxx() const { return _Vec4<T>(x, z, x, x); }
    constexpr _Vec4<T> xzyy() const { return _Vec4<T>(x, z, y, y); }
    constexpr _Vec4<T> xzzz() const { return _Vec4<T>(x, z, z, z); }
    constexpr _Vec4<T> xzww() const { return _Vec4<T>(x, z, w, w); }
    constexpr _Vec4<T> xzxy() const { return _Vec4<T>(x, z, x, y); }
    constexpr _Vec4<T> xzyx() const { return _Vec4<T>(x, z, y, x); }
    constexpr _Vec4<T> xzzy() const { return _Vec4<T>(x, z, z, y); }
    constexpr _Vec4<T> xzyz() const { return _Vec4<T>(x, z, y, z); }
    constexpr _Vec4<T> xzxz() const { return _Vec4<T>(x, z, x, z); }
    constexpr _Vec4<T> xzzx() const { return _Vec4<T>(x, z, z, x); }
    constexpr _Vec4<T> xzxw() const { return _Vec4<T>(x, z, x, w); }
    constexpr _Vec4<T> xzwx() const { return _Vec4<T>(x, z, w, x); }
    constexpr _Vec4<T> xzyw() const { return _Vec4<T>(x, z, y, w); }
    constexpr _Vec4<T> xzwy() const { return _Vec4<T>(x, z, w, y); }
    constexpr _Vec4<T> xzzw() const { return _Vec4<T>(x, z, z, w); }
    constexpr _Vec4<T> xzwz() const { return _Vec4<T>(x, z, w, z); }
    constexpr _Vec4<T> xwxx() const { return _Vec4<T>(x, w, x, x); }
    constexpr _Vec4<T> xwyy() const { return _Vec4<T>(x, w, y, y); }
    constexpr _Vec4<T> xwzz() const { return _Vec4<T>(x, w, z, z); }
    constexpr _Vec4<T> xwww() const { return _Vec4<T>(x, w, w, w); }
    constexpr _Vec4<T> xwxy() const { return _Vec4<T>(x, w, x, y); }
    constexpr _Vec4<T> xwyx() const { return _Vec4<T>(x, w, y, x); }
    constexpr _Vec4<T> xwzy() const { return _Vec4<T>(x, w, z, y); }
    constexpr _Vec4<T> xwyz() const { return _Vec4<T>(x, w, y, z); }
    constexpr _Vec4<T> xwxz() const { return _Vec4<T>(x, w, x, z); }
    constexpr _Vec4<T> xwzx() const { return _Vec4<T>(x, w, z, x); }
    constexpr _Vec4<T> xwxw() const { return _Vec4<T>(x, w, x, w); }
    constexpr _Vec4<T> xwwx() const { return _Vec4<T>(x, w, w, x); }
    constexpr _Vec4<T> xwyw() const { return _Vec4<T>(x, w, y, w); }
    constexpr _Vec4<T> xwwy() const { return _Vec4<T>(x, w, w, y); }
    constexpr _Vec4<T> xwzw() const { return _Vec4<T>(x, w, z, w); }
    constexpr _Vec4<T> xwwz() const { return _Vec4<T>(x, w, w, z); }
    constexpr _Vec4<T> yxxx() const { return _Vec4<T>(y, x, x, x); }
    constexpr _Vec4<T> yxyy() const { return _Vec4<T>(y, x, y, y); }
    constexpr _Vec4<T> yxzz() const { return _Vec4<T>(y, x, z, z); }
    constexpr _Vec4<T> yxww() const { return _Vec4<T>(y, x, w, w); }
    constexpr _Vec4<T> yxxy() const { return _Vec4<T>(y, x, x, y); }
    constexpr _Vec4<T> yxyx() const { return _Vec4<T>(y, x, y, x); }
    constexpr _Vec4<T> yxzy() const { return _Vec4<T>(y, x, z, y); }
    constexpr _Vec4<T> yxyz() const { return _Vec4<T>(y, x, y, z); }
    constexpr _Vec4<T> yxxz() const { return _Vec4<T>(y, x, x, z); }
    constexpr _Vec4<T> yxzx() const { return _Vec4<T>(y, x, z, x); }
    constexpr _Vec4<T> yxxw() const { return _Vec4<T>(y, x, x, w); }
    constexpr _Vec4<T> yxwx() const { return _Vec4<T>(y, x, w, x); }
    constexpr _Vec4<T> yxyw() const { return _Vec4<T>(y, x, y, w); }
    constexpr _Vec4<T> yxwy() const { return _Vec4<T>(y, x, w, y); }
    constexpr _Vec4<T> yxzw() const { return _Vec4<T>(y, x, z, w); }
    constexpr _Vec4<T> yxwz() const { return _Vec4<T>(y, x, w, z); }
    constexpr _Vec4<T> yyxx() const { return _Vec4<T>(y, y, x, x); }
    constexpr _Vec4<T> yyyy() const { return _Vec4<T>(y, y, y, y); }
    constexpr _Vec4<T> yyzz() const { return _Vec4<T>(y, y, z, z); }
    constexpr _Vec4<T> yyww() const { return _Vec4<T>(y, y, w, w); }
    constexpr _Vec4<T> yyxy() const { return _Vec4<T>(y, y, x, y); }
    constexpr _Vec4<T> yyyx() const { return _Vec4<T>(y, y, y, x); }
    constexpr _Vec4<T> yyzy() const { return _Vec4<T>(y, y, z, y); }
    constexpr _Vec4<T> yyyz() const { return _Vec4<T>(y, y, y, z); }
    constexpr _Vec4<T> yyxz() const { return _Vec4<T>(y, y, x, z); }
    constexpr _Vec4<T> yyzx() const { return _Vec4<T>(y, y, z, x); }
    constexpr _Vec4<T> yyxw() const { return _Vec4<T>(y, y, x, w); }
    constexpr _Vec4<T> yywx() const { return _Vec4<T>(y, y, w, x); }
    constexpr _Vec4<T> yyyw() const { return _Vec4<T>(y, y, y, w); }
    constexpr _Vec4<T> yywy() const { return _Vec4<T>(y, y, w, y); }
    constexpr _Vec4<T> yyzw() const { return _Vec4<T>(y, y, z, w); }
    constexpr _Vec4<T> yywz() const { return _Vec4<T>(y, y, w, z); }
    constexpr _Vec4<T> yzxx() const { return _Vec4<T>(y, z, x, x); }
    constexpr _Vec4<T> yzyy() const { return _Vec4<T>(y, z, y, y); }
    constexpr _Vec4<T> yzzz() const { return _Vec4<T>(y, z, z, z); }
    constexpr _Vec4<T> yzww() const { return _Vec4<T>(y, z, w, w); }
    constexpr _Vec4<T> yzxy() const { return _Vec4<T>(y, z, x, y); }
    constexpr _Vec4<T> yzyx() const { return _Vec4<T>(y, z, y, x); }
    constexpr _Vec4<T> yzzy() const { return _Vec4<T>(y, z, z, y); }
    constexpr _Vec4<T> yzyz() const { return _Vec4<T>(y, z, y, z); }
    constexpr _Vec4<T> yzxz() const { return _Vec4<T>(y, z, x, z); }
    constexpr _Vec4<T> yzzx() const { return _Vec4<T>(y, z, z, x); }
    constexpr _Vec4<T> yzxw() const { return _Vec4<T>(y, z, x, w); }
    constexpr _Vec4<T> yzwx() const { return _Vec4<T>(y, z, w, x); }
    constexpr _Vec4<T> yzyw() const { return _Vec4<T>(y, z, y, w); }
    constexpr _Vec4<T> yzwy() const { return _Vec4<T>(y, z, w, y); }
    constexpr _Vec4<T> yzzw() const { return _Vec4<T>(y, z, z, w); }
    constexpr _Vec4<T> yzwz() const { return _Vec4<T>(y, z, w, z); }
    constexpr _Vec4<T> ywxx() const { return _Vec4<T>(y, w, x, x); }
    constexpr _Vec4<T> ywyy() const { return _Vec4<T>(y, w, y, y); }
    constexpr _Vec4<T> ywzz() const { return _Vec4<T>(y, w, z, z); }
    constexpr _Vec4<T> ywww() const { return _Vec4<T>(y, w, w, w); }
    constexpr _Vec4<T> ywxy() const { return _Vec4<T>(y, w, x, y); }
    constexpr _Vec4<T> ywyx() const { return _Vec4<T>(y, w, y, x); }
    constexpr _Vec4<T> ywzy() const { return _Vec4<T>(y, w, z, y); }
    constexpr _Vec4<T> ywyz() const { return _Vec4<T>(y, w, y, z); }
    constexpr _Vec4<T> ywxz() const { return _Vec4<T>(y, w, x, z); }
    constexpr _Vec4<T> ywzx() const { return _Vec4<T>(y, w, z, x); }
    constexpr _Vec4<T> ywxw() const { return _Vec4<T>(y, w, x, w); }
    constexpr _Vec4<T> ywwx() const { return _Vec4<T>(y, w, w, x); }
    constexpr _Vec4<T> ywyw() const { return _Vec4<T>(y, w, y, w); }
    constexpr _Vec4<T> ywwy() const { return _Vec4<T>(y, w, w, y); }
    constexpr _Vec4<T> ywzw() const { return _Vec4<T>(y, w, z, w); }
    constexpr _Vec4<T> ywwz() const { return _Vec4<T>(y, w, w, z); }
    constexpr _Vec4<T> zxxx() const { return _Vec4<T>(z, x, x, x); }
    constexpr _Vec4<T> zxyy() const { return _Vec4<T>(z, x, y, y); }
    constexpr _Vec4<T> zxzz() const { return _Vec4<T>(z, x, z, z); }
    constexpr _Vec4<T> zxww() const { return _Vec4<T>(z, x, w, w); }
    constexpr _Vec4<T> zxxy() const { return _Vec4<T>(z, x, x, y); }
    constexpr _Vec4<T> zxyx() const { return _Vec4<T>(z, x, y, x); }
    constexpr _Vec4<T> zxzy() const { return _Vec4<T>(z, x, z, y); }
    constexpr _Vec4<T> zxyz() const { return _Vec4<T>(z, x, y, z); }
    constexpr _Vec4<T> zxxz() const { return _Vec4<T>(z, x, x, z); }
    constexpr _Vec4<T> zxzx() const { return _Vec4<T>(z, x, z, x); }
    constexpr _Vec4<T> zxxw() const { return _Vec4<T>(z, x, x, w); }
    constexpr _Vec4<T> zxwx() const { return _Vec4<T>(z, x, w, x); }
    constexpr _Vec4<T> zxyw() const { return _Vec4<T>(z, x, y, w); }
    constexpr _Vec4<T> zxwy() const { return _Vec4<T>(z, x, w, y); }
    constexpr _Vec4<T> zxzw() const { return _Vec4<T>(z, x, z, w); }
    constexpr _Vec4<T> zxwz() const { return _Vec4<T>(z, x, w, z); }
    constexpr _Vec4<T> zyxx() const { return _Vec4<T>(z, y, x, x); }
    constexpr _Vec4<T> zyyy() const { return _Vec4<T>(z, y, y, y); }
    constexpr _Vec4<T> zyzz() const { return _Vec4<T>(z, y, z, z); }
    constexpr _Vec4<T> zyww() const { return _Vec4<T>(z, y, w, w); }
    constexpr _Vec4<T> zyxy() const { return _Vec4<T>(z, y, x, y); }
    constexpr _Vec4<T> zyyx() const { return _Vec4<T>(z, y, y, x); }
    constexpr _Vec4<T> zyzy() const { return _Vec4<T>(z, y, z, y); }
    constexpr _Vec4<T> zyyz() const { return _Vec4<T>(z, y, y, z); }
    constexpr _Vec4<T> zyxz() const { return _Vec4<T>(z, y, x, z); }
    constexpr _Vec4<T> zyzx() const { return _Vec4<T>(z, y, z, x); }
    constexpr _Vec4<T> zyxw() const { return _Vec4<T>(z, y, x, w); }
    constexpr _Vec4<T> zywx() const { return _Vec4<T>(z, y, w, x); }
    constexpr _Vec4<T> zyyw() const { return _Vec4<T>(z, y, y, w); }
    constexpr _Vec4<T> zywy() const { return _Vec4<T>(z, y, w, y); }
    constexpr _Vec4<T> zyzw() const { return _Vec4<T>(z, y, z, w); }
    constexpr _Vec4<T> zywz() const { return _Vec4<T>(z, y, w, z); }
    constexpr _Vec4<T> zzxx() const { return _Vec4<T>(z, z, x, x); }
    constexpr _Vec4<T> zzyy() const { return _Vec4<T>(z, z, y, y); }
    constexpr _Vec4<T> zzzz() const { return _Vec4<T>(z, z, z, z); }
    constexpr _Vec4<T> zzww() const { return _Vec4<T>(z, z, w, w); }
    constexpr _Vec4<T> zzxy() const { return _Vec4<T>(z, z, x, y); }
    constexpr _Vec4<T> zzyx() const { return _Vec4<T>(z, z, y, x); }
    constexpr _Vec4<T> zzzy() const { return _Vec4<T>(z, z, z, y); }
    constexpr _Vec4<T> zzyz() const { return _Vec4<T>(z, z, y, z); }
    constexpr _Vec4<T> zzxz() const { return _Vec4<T>(z, z, x, z); }
    constexpr _Vec4<T> zzzx() const { return _Vec4<T>(z, z, z, x); }
    constexpr _Vec4<T> zzxw() const { return _Vec4<T>(z, z, x, w); }
    constexpr _Vec4<T> zzwx() const { return _Vec4<T>(z, z, w, x); }
    constexpr _Vec4<T> zzyw() const { return _Vec4<T>(z, z, y, w); }
    constexpr _Vec4<T> zzwy() const { return _Vec4<T>(z, z, w, y); }
    constexpr _Vec4<T> zzzw() const { return _Vec4<T>(z, z, z, w); }
    constexpr _Vec4<T> zzwz() const { return _Vec4<T>(z, z, w, z); }
    constexpr _Vec4<T> zwxx() const { return _Vec4<T>(z, w, x, x); }
    constexpr _Vec4<T> zwyy() const { return _Vec4<T>(z, w, y, y); }
    constexpr _Vec4<T> zwzz() const { return _Vec4<T>(z, w, z, z); }
    constexpr _Vec4<T> zwww() const { return _Vec4<T>(z, w, w, w); }
    constexpr _Vec4<T> zwxy() const { return _Vec4<T>(z, w, x, y); }
    constexpr _Vec4<T> zwyx() const { return _Vec4<T>(z, w, y, x); }
    constexpr _Vec4<T> zwzy() const { return _Vec4<T>(z, w, z, y); }
    constexpr _Vec4<T> zwyz() const { return _Vec4<T>(z, w, y, z); }
    constexpr _Vec4<T> zwxz() const { return _Vec4<T>(z, w, x, z); }
    constexpr _Vec4<T> zwzx() const { return _Vec4<T>(z, w, z, x); }
    constexpr _Vec4<T> zwxw() const { return _Vec4<T>(z, w, x, w); }
    constexpr _Vec4<T> zwwx() const { return _Vec4<T>(z, w, w, x); }
    constexpr _Vec4<T> zwyw() const { return _Vec4<T>(z, w, y, w); }
    constexpr _Vec4<T> zwwy() const { return _Vec4<T>(z, w, w, y); }
    constexpr _Vec4<T> zwzw() const { return _Vec4<T>(z, w, z, w); }
    constexpr _Vec4<T> zwwz() const { return _Vec4<T>(z, w, w, z); }
    constexpr _Vec4<T> wxxx() const { return _Vec4<T>(w, x, x, x); }
    constexpr _Vec4<T> wxyy() const { return _Vec4<T>(w, x, y, y); }
    constexpr _Vec4<T> wxzz() const { return _Vec4<T>(w, x, z, z); }
    constexpr _Vec4<T> wxww() const { return _Vec4<T>(w, x, w, w); }
    constexpr _Vec4<T> wxxy() const { return _Vec4<T>(w, x, x, y); }
    constexpr _Vec4<T> wxyx() const { return _Vec4<T>(w, x, y, x); }
    constexpr _Vec4<T> wxzy() const { return _Vec4<T>(w, x, z, y); }
    constexpr _Vec4<T> wxyz() const { return _Vec4<T>(w, x, y, z); }
    constexpr _Vec4<T> wxxz() const { return _Vec4<T>(w, x, x, z); }
    constexpr _Vec4<T> wxzx() const { return _Vec4<T>(w, x, z, x); }
    constexpr _Vec4<T> wxxw() const { return _Vec4<T>(w, x, x, w); }
    constexpr _Vec4<T> wxwx() const { return _Vec4<T>(w, x, w, x); }
    constexpr _Vec4<T> wxyw() const { return _Vec4<T>(w, x, y, w); }
    constexpr _Vec4<T> wxwy() const { return _Vec4<T>(w, x, w, y); }
    constexpr _Vec4<T> wxzw() const { return _Vec4<T>(w, x, z, w); }
    constexpr _Vec4<T> wxwz() const { return _Vec4<T>(w, x, w, z); }
    constexpr _Vec4<T> wyxx() const { return _Vec4<T>(w, y, x, x); }
    constexpr _Vec4<T> wyyy() const { return _Vec4<T>(w, y, y, y); }
    constexpr _Vec4<T> wyzz() const { return _Vec4<T>(w, y, z, z); }
    constexpr _Vec4<T> wyww() const { return _Vec4<T>(w, y, w, w); }
    constexpr _Vec4<T> wyxy() const { return _Vec4<T>(w, y, x, y); }
    constexpr _Vec4<T> wyyx() const { return _Vec4<T>(w, y, y, x); }
    constexpr _Vec4<T> wyzy() const { return _Vec4<T>(w, y, z, y); }
    constexpr _Vec4<T> wyyz() const { return _Vec4<T>(w, y, y, z); }
    constexpr _Vec4<T> wyxz() const { return _Vec4<T>(w, y, x, z); }
    constexpr _Vec4<T> wyzx() const { return _Vec4<T>(w, y, z, x); }
    constexpr _Vec4<T> wyxw() const { return _Vec4<T>(w, y, x, w); }
    constexpr _Vec4<T> wywx() const { return _Vec4<T>(w, y, w, x); }
    constexpr _Vec4<T> wyyw() const { return _Vec4<T>(w, y, y, w); }
    constexpr _Vec4<T> wywy() const { return _Vec4<T>(w, y, w, y); }
    constexpr _Vec4<T> wyzw() const { return _Vec4<T>(w, y, z, w); }
    constexpr _Vec4<T> wywz() const { return _Vec4<T>(w, y, w, z); }
    constexpr _Vec4<T> wzxx() const { return _Vec4<T>(w, z, x, x); }
    constexpr _Vec4<T> wzyy() const { return _Vec4<T>(w, z, y, y); }
    constexpr _Vec4<T> wzzz() const { return _Vec4<T>(w, z, z, z); }
    constexpr _Vec4<T> wzww() const { return _Vec4<T>(w, z, w, w); }
    constexpr _Vec4<T> wzxy() const { return _Vec4<T>(w, z, x, y); }
    constexpr _Vec4<T> wzyx() const { return _Vec4<T>(w, z, y, x); }
    constexpr _Vec4<T> wzzy() const { return _Vec4<T>(w, z, z, y); }
    constexpr _Vec4<T> wzyz() const { return _Vec4<T>(w, z, y, z); }
    constexpr _Vec4<T> wzxz() const { return _Vec4<T>(w, z, x, z); }
    constexpr _Vec4<T> wzzx() const { return _Vec4<T>(w, z, z, x); }
    constexpr _Vec4<T> wzxw() const { return _Vec4<T>(w, z, x, w); }
    constexpr _Vec4<T> wzwx() const { return _Vec4<T>(w, z, w, x); }
    constexpr _Vec4<T> wzyw() const { return _Vec4<T>(w, z, y, w); }
    constexpr _Vec4<T> wzwy() const { return _Vec4<T>(w, z, w, y); }
    constexpr _Vec4<T> wzzw() const { return _Vec4<T>(w, z, z, w); }
    constexpr _Vec4<T> wzwz() const { return _Vec4<T>(w, z, w, z); }
    constexpr _Vec4<T> wwxx() const { return _Vec4<T>(w, w, x, x); }
    constexpr _Vec4<T> wwyy() const { return _Vec4<T>(w, w, y, y); }
    constexpr _Vec4<T> wwzz() const { return _Vec4<T>(w, w, z, z); }
    constexpr _Vec4<T> wwww() const { return _Vec4<T>(w, w, w, w); }
    constexpr _Vec4<T> wwxy() const { return _Vec4<T>(w, w, x, y); }
    constexpr _Vec4<T> wwyx() const { return _Vec4<T>(w, w, y, x); }
    constexpr _Vec4<T> wwzy() const { return _Vec4<T>(w, w, z, y); }
    constexpr _Vec4<T> wwyz() const { return _Vec4<T>(w, w, y, z); }
    constexpr _Vec4<T> wwxz() const { return _Vec4<T>(w, w, x, z); }
    constexpr _Vec4<T> wwzx() const { return _Vec4<T>(w, w, z, x); }
    constexpr _Vec4<T> wwxw() const { return _Vec4<T>(w, w, x, w); }
    constexpr _Vec4<T> wwwx() const { return _Vec4<T>(w, w, w, x); }
    constexpr _Vec4<T> wwyw() const { return _Vec4<T>(w, w, y, w); }
    constexpr _Vec4<T> wwwy() const { return _Vec4<T>(w, w, w, y); }
    constexpr _Vec4<T> wwzw() const { return _Vec4<T>(w, w, z, w); }
    constexpr _Vec4<T> wwwz() const { return _Vec4<T>(w, w, w, z); }

  public:
    constexpr T &operator[](const size_t index) { return reinterpret_cast<T*>(this)[index]; }
    constexpr const T &operator[](const size_t index) const { return reinterpret_cast<const T*>(this)[index]; }

  public:
    static constexpr const size_t SIZE = 4;

  public:
    static const _Vec4<T> ZERO;
    static const _Vec4<T> ONE;
    static const _Vec4<T> INF;
    static const _Vec4<T> X;
    static const _Vec4<T> Y;
    static const _Vec4<T> Z;
    static const _Vec4<T> W;
  };


  template<Number T>
  constexpr const _Vec4<T> _Vec4<T>::ZERO = _Vec4<T>(T(0), T(0), T(0), T(0));
  template<Number T>
  constexpr const _Vec4<T> _Vec4<T>::ONE = _Vec4<T>(T(1), T(1), T(1), T(1));
  template<Number T>
  constexpr const _Vec4<T> _Vec4<T>::INF = _Vec4<T>(std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity());
  template<Number T>
  constexpr const _Vec4<T> _Vec4<T>::X = _Vec4<T>(T(1), T(0), T(0), T(0));
  template<Number T>
  constexpr const _Vec4<T> _Vec4<T>::Y = _Vec4<T>(T(0), T(1), T(0), T(0));
  template<Number T>
  constexpr const _Vec4<T> _Vec4<T>::Z = _Vec4<T>(T(0), T(0), T(1), T(0));
  template<Number T>
  constexpr const _Vec4<T> _Vec4<T>::W = _Vec4<T>(T(0), T(0), T(0), T(1));


  // =====================
  // = Vector4 Functions =
  // =====================


  template<Number T>
  constexpr T length_squared(const _Vec4<T> &v) {
    return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
  }
  

  template<Number T>
  constexpr T length(const _Vec4<T> &v) {
    return sqrt(length_squared(v));
  }


  template<Number T>
  constexpr T distance_squared(const _Vec4<T> &a, const _Vec4<T> &b) {
    return length_squared(b - a);
  }


  template<Number T>
  constexpr T distance(const _Vec4<T> &a, const _Vec4<T> &b) {
    return length(b - a);
  }
  

  template<Number T>
  constexpr _Vec4<T> normalized(const _Vec4<T> &v) {
    return v / length(v);
  }
    

  template<Number T>
  constexpr T dot(const _Vec4<T> &a, const _Vec4<T> &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  }


  template<Number T>
  constexpr _Vec3<T> homogeneous_projection(const _Vec4<T> &v) {
    return _Vec3<T>(v.x, v.y, v.z) / v.w;
  }
  

  // =====================
  // = Vector4 Operators =
  // =====================


  template<Number T>
  constexpr _Vec4<T> operator+(const _Vec4<T> &a, const _Vec4<T> &b) {
    _Vec4<T> res(a);
    res += b;
    return res;
  }


  template<Number T>
  constexpr _Vec4<T> &operator+=(_Vec4<T> &a, const _Vec4<T> &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
    return a;
  }


  template<Number T>
  constexpr _Vec4<T> operator-(const _Vec4<T> &a, const _Vec4<T> &b) {
    _Vec4<T> res(a);
    res -= b;
    return res;
  }


  template<Number T>
  constexpr _Vec4<T> &operator-=(_Vec4<T> &a, const _Vec4<T> &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
    return a;
  }


  template<Number T>
  constexpr _Vec4<T> operator-(const _Vec4<T> &a) {
    return _Vec4<T>(
      -a.x, -a.y, -a.z, -a.w    
    );
  }


  template<Number T>
  constexpr _Vec4<T> operator*(const _Vec4<T> &a, const _Vec4<T> &b) {
    _Vec4<T> res(a);
    res *= b;
    return res;
  }


  template<Number T>
  constexpr _Vec4<T> &operator*=(_Vec4<T> &a, const _Vec4<T> &b) {
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
    return a;
  }


  template<Number T>
  constexpr _Vec4<T> operator/(const _Vec4<T> &a, const _Vec4<T> &b) {
    _Vec4<T> res(a);
    res /= b;
    return res;
  }


  template<Number T>
  constexpr _Vec4<T> &operator/=(_Vec4<T> &a, const _Vec4<T> &b) {
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
    a.w /= b.w;
    return a;
  }


  template<Number T>
  constexpr _Vec4<T> operator*(const T s, const _Vec4<T> &v) {
    return _Vec4<T>(
      s * v.x,
      s * v.y,
      s * v.z,
      s * v.w
    );
  }


  template<Number T>
  constexpr _Vec4<T> operator*(const _Vec4<T> &v, const T s) {
    return _Vec4<T>(
      v.x * s,
      v.y * s,
      v.z * s,
      v.w * s
    );
  }


  template<Number T>
  constexpr _Vec4<T> &operator*=(_Vec4<T> &v, const T s) {
    v = v * s;
    return v;
  }


  template<Number T>
  constexpr _Vec4<T> operator/(const _Vec4<T> &v, const T s) {
    _Vec4<T> res(v);
    res /= s;
    return res;
  }


  template<Number T>
  constexpr _Vec4<T> &operator/=(_Vec4<T> &v, const T s) {
    const T scale = T(1) / s;
    return v *= scale;
  }


  template<Number T>
  constexpr bool operator==(const _Vec4<T> &a, const _Vec4<T> &b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
  }


  template<Number T>
  constexpr bool operator!=(const _Vec4<T> &a, const _Vec4<T> &b) {
    return !(a == b);
  }



  // ================
  // = Type aliases =
  // ================

  typedef _Vec2<float> Vec2;
  typedef _Vec2<double> Vec2d;
  typedef _Vec2<int> Vec2i;
  typedef _Vec2<long> Vec2l;
  typedef _Vec2<bool> Vec2b;

  typedef _Vec3<float> Vec3;
  typedef _Vec3<double> Vec3d;
  typedef _Vec3<int> Vec3i;
  typedef _Vec3<long> Vec3l;
  typedef _Vec3<bool> Vec3b;

  typedef _Vec4<float> Vec4;
  typedef _Vec4<double> Vec4d;
  typedef _Vec4<int> Vec4i;
  typedef _Vec4<long> Vec4l;
  typedef _Vec4<bool> Vec4b;
}
