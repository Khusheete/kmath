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


namespace kmath {

  struct Vec3;
  struct Vec4;
  struct Quat;
  struct DQuat;
  struct Mat3;
  struct Mat4;


  struct Vec3 {
    double x, y, z;

    Vec3 &normalize();

    static const Vec3 ZERO;
    static const Vec3 ONE;
    static const Vec3 INF;
    static const Vec3 X;
    static const Vec3 Y;
    static const Vec3 Z;
  };


  struct Vec4 {
    double x, y, z, w;

    Vec4 &normalize();
    
    operator Quat() const;

    static const Vec4 ZERO;
    static const Vec4 ONE;
    static const Vec4 INF;
    static const Vec4 X;
    static const Vec4 Y;
    static const Vec4 Z;
    static const Vec4 W;
  };


  struct Quat {
    // q = x . i + y . j + z . k + w
    double x, y, z, w;

    static Quat from_axis_angle(const Vec3 &axis, const double angle); // axis must be a unit vector

    Quat &normalize();

    operator const Vec4&() const;
    operator Vec4&();
    operator Vec4() const;
    operator const Vec3&() const;
    operator Vec3&();
    operator Vec3() const;

    Quat(const double x, const double y, const double z, const double w);
    Quat(const Vec3 &v, const double w);
    Quat(const Vec3 &v);

    static const Quat ZERO;
    static const Quat IDENTITY;
    static const Quat X;
    static const Quat Y;
    static const Quat Z;
    static const Quat W;
  };


  struct DQuat {
    double yz, zx, xy;
    double w;
    double dx, dy, dz;
    double dxyz;

    static DQuat from_point(const Vec3 &point);
    static DQuat from_line(const Vec3 &point, const Vec3 &direction);
    
    static DQuat from_axis_angle(const Vec3 &axis, const double angle);
    static DQuat from_translation(const Vec3 &delta);
    static DQuat from_axis_angle_translation(const Vec3 &axis, const double angle, const Vec3 &translation);
    static DQuat from_rotation(const Quat &rotation);
    static DQuat from_rotation_translation(const Quat &rotation, const Vec3 &translation);
    static DQuat from_screw_coordinates(const Vec3 &direction, const Vec3 &moment, const double &angle, const double &translation);

    Quat &real_part();
    const Quat &real_part() const;
    Quat &dual_part();
    const Quat &dual_part() const;
    
    Quat get_rotation() const; // Returns the rotation quaternion linked with this (unit) dual-quaternion
    Vec3 get_translation() const; // Returns the translation component of this (unit) dual-quaternion

    Vec3 get_point() const; // If this dual-quaternion represents a point, returns the coordinates of this point

    void get_screw_coordinates(Vec3 &direction, Vec3 &moment, double &angle, double &translation) const; // Returns the screw coordinates (I, m, theta, d)

    DQuat &normalize(); // Tries to normalize this dual-quaternion. Note that it may still not be a unit dual quaternion after this operation.

    DQuat(const double yz, const double zx, const double xy, const double w, const double dx, const double dy, const double dz, const double dxyz);
    DQuat(const Quat &real, const Quat &dual);

    static const DQuat IDENTITY;
  };


  struct Mat3 {
    Vec3 x, y, z;
  };

  typedef Mat3 Basis;


  struct Mat4 {
    Vec4 x, y, z, w;
  };


  // ==========================
  // = Utility math functions =
  // ==========================

  double dot(const Vec3 &a, const Vec3 &b);
  double dot(const Vec4 &a, const Vec4 &b);
  Vec3 cross(const Vec3 &a, const Vec3 &b);

  double length(const Vec3 &a);
  double length_squared(const Vec3 &a);
  double length(const Vec4 &a);
  double length_squared(const Vec4 &a);
  double length(const Quat &a);
  double length_squared(const Quat &a);

  bool is_approx_zero(const double a);
  bool is_approx_zero(const Vec3 &a);
  bool is_approx_zero(const Vec4 &a);
  bool is_approx_zero(const Quat &a);

  double inv_lerp(const double a, const double b, const double x);
  
  template<class S>
  S lerp(const S &a, const S &b, const double t) {
    return (1.0 - t) * a + t * b;
  }

  template<class S>
  S normalized(const S &a) {
    return a / length(a);
  }

  template<class S>
  bool is_approx(const S &a, const S &b) {
    return is_approx_zero(b - a);
  }
  

  // Quaternion specific functions

  Quat conjugate(const Quat &a);
  Quat conjugate(const Quat &p, const Quat &q); // Returns the conjugation of q by p.
  Quat conjugate_unit(const Quat &p, const Quat &q); // Returns the conjugation of q by the unit quaternion p.
  Quat inverse(const Quat &a); // Note that for a unit quaternion a, `inverse(a) = conjugate(a)`.

  Quat exp(const Quat &a);
  Quat ln(const Quat &a);
  Quat pow(const Quat &a, const double b);

  Quat slerp(const Quat &a, const Quat &b, const double t); // Spherical interpolation
  

  // Dual-Quaternion specific functions
  
  DQuat dual_conjugate(const DQuat &a);
  DQuat quat_conjugate(const DQuat &a);
  DQuat dquat_conjugate(const DQuat &a);
  DQuat conjugate_unit(const DQuat &p, const DQuat &q);
  DQuat inverse(const DQuat &a);

  DQuat seplerp(const DQuat &a, const DQuat &b, const double t); // Separate lerp (SEP(LERP))
  DQuat sclerp(const DQuat &a, const DQuat &b, const double t); // Screw linear interpolation (ScLERP)
  DQuat kenlerp(const DQuat &a, const DQuat &b, const double t, const double beta); // Hybrid SEP(LERP) - ScLerp interpolation


  // ==================
  // = Math operators =
  // ==================


  Vec3 operator+(const Vec3 &a, const Vec3 &b);
  Vec3 &operator+=(Vec3 &a, const Vec3 &b);
  Vec3 operator-(const Vec3 &a, const Vec3 &b);
  Vec3 &operator-=(Vec3 &a, const Vec3 &b);
  Vec3 operator-(const Vec3 &a);
  Vec3 operator*(const double a, const Vec3 &b);
  Vec3 operator*(const Vec3 &a, const double b);
  Vec3 &operator*=(Vec3 &a, const double b);
  Vec3 operator/(const Vec3 &a, const double b);
  Vec3 &operator/=(Vec3 &a, const double b);

  Vec4 operator+(const Vec4 &a, const Vec4 &b);
  Vec4 &operator+=(Vec4 &a, const Vec4 &b);
  Vec4 operator-(const Vec4 &a, const Vec4 &b);
  Vec4 &operator-=(Vec4 &a, const Vec4 &b);
  Vec4 operator-(const Vec4 &a);
  Vec4 operator*(const double a, const Vec4 &b);
  Vec4 operator*(const Vec4 &a, const double b);
  Vec4 &operator*=(Vec4 &a, const double b);
  Vec4 operator/(const Vec4 &a, const double b);
  Vec4 &operator/=(Vec4 &a, const double b);

  Quat operator+(const Quat &a, const Quat &b);
  Quat &operator+=(Quat &a, const Quat &b);
  Quat operator-(const Quat &a, const Quat &b);
  Quat &operator-=(Quat &a, const Quat &b);
  Quat operator-(const Quat &a);
  Quat operator*(const double a, const Quat &b);
  Quat operator*(const Quat &a, const double b);
  Quat &operator*=(Quat &a, const double b);
  Quat operator/(const Quat &a, const double b);
  Quat &operator/=(Quat &a, const double b);
  Quat operator*(const Quat &a, const Quat &b);
  Quat &operator*=(Quat &a, const Quat &b);

  Vec3 operator*(const Mat3 &a, const Vec3 &b);
  Vec3 operator*(const Vec3 &a, const Mat3 &b);

  Mat3 operator*(const Mat3 &a, const Mat3 &b);
  Mat3 &operator*=(Mat3 &a, const Mat3 &b);

  Vec4 operator*(const Mat4 &a, const Vec4 &b);
  Vec4 operator*(const Vec4 &a, const Mat4 &b);

  Mat4 operator*(const Mat4 &a, const Mat4 &b);
  Mat4 &operator*=(Mat4 &a, const Mat4 &b);

  DQuat operator+(const DQuat &a, const DQuat &b);
  DQuat &operator+=(DQuat &a, const DQuat &b);
  DQuat operator-(const DQuat &a, const DQuat &b);
  DQuat &operator-=(DQuat &a, const DQuat &b);
  DQuat operator-(const DQuat &a);
  DQuat operator*(const double a, const DQuat &b);
  DQuat operator*(const DQuat &a, const double b);
  DQuat &operator*=(DQuat &a, const double b);
  DQuat operator/(const DQuat &a, const double b);
  DQuat &operator/=(DQuat &a, const double b);
  DQuat operator*(const DQuat &a, const DQuat &b);
  DQuat &operator*=(DQuat &a, const DQuat &b);


  // ===================
  // = Print operators =
  // ===================
  
  std::ostream &operator<<(std::ostream &os, const Vec3 &v);
  std::ostream &operator<<(std::ostream &os, const Vec4 &v);
  std::ostream &operator<<(std::ostream &os, const Quat &q);
  std::ostream &operator<<(std::ostream &os, const DQuat &q);
}
