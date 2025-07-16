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


#include "kmath.hpp"

#include <cmath>
#include <limits>


#define EPSILON 0.00001
#define EPSILON2 (EPSILON * EPSILON)


namespace kmath {

  // =========================
  // = Struct Implementation =
  // =========================

  const Vec3 Vec3::ZERO = { 0.0, 0.0, 0.0 };
  const Vec3 Vec3::ONE  = { 1.0, 1.0, 1.0};
  const Vec3 Vec3::INF  = { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
  const Vec3 Vec3::X    = { 1.0, 0.0, 0.0 };
  const Vec3 Vec3::Y    = { 0.0, 1.0, 0.0 };
  const Vec3 Vec3::Z    = { 0.0, 0.0, 1.0 };

  const Vec4 Vec4::ZERO = { 0.0, 0.0, 0.0, 0.0 };
  const Vec4 Vec4::ONE  = { 1.0, 1.0, 1.0, 1.0 };
  const Vec4 Vec4::INF  = { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
  const Vec4 Vec4::X    = { 1.0, 0.0, 0.0, 0.0 };
  const Vec4 Vec4::Y    = { 0.0, 1.0, 0.0, 0.0 };
  const Vec4 Vec4::Z    = { 0.0, 0.0, 1.0, 0.0 };
  const Vec4 Vec4::W    = { 0.0, 0.0, 0.0, 1.0 };

  const Quat Quat::ZERO     = { 0.0, 0.0, 0.0, 0.0 };
  const Quat Quat::IDENTITY = { 0.0, 0.0, 0.0, 1.0 };
  const Quat Quat::X        = { 1.0, 0.0, 0.0, 0.0 };
  const Quat Quat::Y        = { 0.0, 1.0, 0.0, 0.0 };
  const Quat Quat::Z        = { 0.0, 0.0, 1.0, 0.0 };
  const Quat Quat::W        = { 0.0, 0.0, 0.0, 1.0 };

  const DQuat DQuat::IDENTITY = { Quat::IDENTITY, Quat::ZERO };


  Quat Quat::from_axis_angle(const Vec3 &axis, const double angle) {
    return {
      std::sin(angle / 2.0) * axis,
      std::cos(angle / 2.0),
    };
  }
  
  Quat::Quat(const double x, const double y, const double z, const double w) : x(x), y(y), z(z), w(w) {}

  Quat::Quat(const Vec3 &v, const double w) : x(v.x), y(v.y), z(v.z), w(w) {}

  Quat::Quat(const Vec3 &v) : x(v.x), y(v.y), z(v.z), w(0.0) {}

  DQuat::DQuat(const double yz, const double zx, const double xy, const double w, const double dx, const double dy, const double dz, const double dxyz)
    : yz(yz), zx(zx), xy(xy), w(w), dx(dx), dy(dy), dz(dz), dxyz(dxyz) {}

  DQuat::DQuat(const Quat &real, const Quat &dual)
    : yz(real.x), zx(real.y), xy(real.z), w(real.w), dx(dual.x), dy(dual.y), dz(dual.z), dxyz(dual.w) {}


  Vec4::operator Quat() const {
    return { x, y, z, w };
  }


  Quat::operator const Vec4&() const {
    return *reinterpret_cast<const Vec4*>(this);
  }


  Quat::operator Vec4&() {
    return *reinterpret_cast<Vec4*>(this);
  }


  Quat::operator Vec4() const {
    return Vec4 {
      x, y, z, w
    };
  }


  Quat::operator const Vec3&() const {
    return *reinterpret_cast<const Vec3*>(this);
  }


  Quat::operator Vec3&() {
    return *reinterpret_cast<Vec3*>(this);
  }


  Quat::operator Vec3() const {
    return { this->x, this->y, this->z };
  }


  DQuat DQuat::from_point(const Vec3 &point) {
    return { 0.0, 0.0, 0.0, 1.0, point.x, point.y, point.z, 0.0 };
  }


  DQuat DQuat::from_line(const Vec3 &point, const Vec3 &direction) {
    double dir_mag2 = length_squared(point);
    // (l, m) is the Plücker coordinates of the line defined by (point, direction)
    Vec3 l = point;
    if (!is_approx(dir_mag2, 1.0)) {
      l /= std::sqrt(dir_mag2);
    }

    Vec3 m = cross(point, l);

    return { {l, 0.0}, {m, 0.0} };
  }
  

  DQuat DQuat::from_axis_angle(const Vec3 &axis, const double angle) {
    return { Quat::from_axis_angle(axis, angle), Quat::ZERO };
  }


  DQuat DQuat::from_translation(const Vec3 &translation) {
    return { Quat::IDENTITY, {0.5 * translation, 0.0} };
  }


  DQuat DQuat::from_axis_angle_translation(const Vec3 &axis, const double angle, const Vec3 &translation) {
    Quat rot = Quat::from_axis_angle(axis, angle);
    Quat trans = {0.5 * translation, 0.0};
    return { rot, trans * rot };
  }


  DQuat DQuat::from_rotation(const Quat &rotation) {
    return { rotation, Quat::ZERO };
  }


  DQuat DQuat::from_rotation_translation(const Quat &rotation, const Vec3 &translation) {
    Quat trans = {0.5 * translation, 0.0};
    return { rotation, trans * rotation };
  }
  

  DQuat DQuat::from_screw_coordinates(const Vec3 &direction, const Vec3 &moment, const double &angle, const double &translation) {
    if (!is_approx_zero(angle)) {
      double cos_a = std::cos(angle / 2.0);
      double sin_a = std::sin(angle / 2.0);
      return { {sin_a * direction, cos_a}, {sin_a * moment + (0.5 * translation * cos_a) * direction, -0.5 * translation * sin_a} };
    } else {
      return DQuat::from_translation(translation * direction);
    }
  }


  Quat &DQuat::real_part() {
    return *reinterpret_cast<Quat*>(this);
  }


  Quat &DQuat::dual_part() {
    return *(1 + reinterpret_cast<Quat*>(this));
  }


  const Quat &DQuat::real_part() const {
    return *reinterpret_cast<const Quat*>(this);
  }


  const Quat &DQuat::dual_part() const {
    return *(1 + reinterpret_cast<const Quat*>(this));
  }
  

  Quat DQuat::get_rotation() const {
    return { yz, zx, xy, w };
  }


  Vec3 DQuat::get_translation() const {
    const Quat &real = real_part();
    const Quat &dual = dual_part();
    Quat translation = 2.0 * dual * conjugate(real);
    return { translation.x, translation.y, translation.z };
  }

  Vec3 DQuat::get_point() const {
    return { dx, dy, dz };
  }


  void DQuat::get_screw_coordinates(Vec3 &direction, Vec3 &moment, double &angle, double &translation) const {
    angle = 2.0 * std::acos(w);
    if (!is_approx_zero(angle)) {
      double inv_sin_a = std::sin(0.5 * angle);
      direction = inv_sin_a * (Vec3&)real_part();
      translation = -2.0 * w * inv_sin_a;
      moment = inv_sin_a * (Vec3&)dual_part() - 0.5 * translation * w * inv_sin_a * direction;
    } else {
      direction = get_translation();
      translation = length(direction);
      if (!is_approx_zero(translation)) { // Normalize translation direction
        direction /= translation;
      } else {
        direction = Vec3::ZERO;
      }
      moment = Vec3::INF;
    }
  }

  
  Vec3 &Vec3::normalize() {
    double len = length(*this);
    *this /= len;
    return *this;
  }

  Vec4 &Vec4::normalize() {
    double len = length(*this);
    *this /= len;
    return *this;
  }
  

  Quat &Quat::normalize() {
    double len = length(*this);
    *this /= len;
    return *this;
  }


  DQuat &DQuat::normalize() {
    double magnitude = length(real_part());
    *this /= magnitude;
    return *this;
  }


  // ==========================
  // = Utility math functions =
  // ==========================


  double dot(const Vec3 &a, const Vec3 &b) {
    return a.x * b.x + a.y * b.x + a.z * b.z;
  }

  
  double dot(const Vec4 &a, const Vec4 &b) {
    return a.x * b.x + a.y * b.x + a.z * b.z + a.w * b.w;
  }


  Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return Vec3 {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
    };
  }


  double length(const Vec3 &a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  }


  double length_squared(const Vec3 &a) {
    return a.x * a.x + a.y * a.y + a.z * a.z;
  }


  double length(const Vec4 &a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
  }


  double length_squared(const Vec4 &a) {
    return a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w;
  }


  double length(const Quat &a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
  }


  double length_squared(const Quat &a) {
    return a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w;
  }


  bool is_approx_zero(const double a) {
    return a < EPSILON;
  }


  bool is_approx_zero(const Vec3 &a) {
    return length_squared(a) < EPSILON2;
  }


  bool is_approx_zero(const Vec4 &a) {
    return length_squared(a) < EPSILON2;
  }


  bool is_approx_zero(const Quat &a) {
    return length_squared(a) < EPSILON2;
  }


  double inv_lerp(const double a, const double b, const double x) {
    return (x - a) / (b - a);
  }


  // Quaternion specific functions


  Quat conjugate(const Quat &a) {
    return (Quat){ -a.x, -a.y, -a.z, a.w };
  }


  Quat conjugate(const Quat &p, const Quat &q) {
    return p * q * inverse(q);
  }


  Quat conjugate_unit(const Quat &p, const Quat &q) {
    return p * q * conjugate(p);
  }


  Quat inverse(const Quat &a) {
    return conjugate(a) / length_squared(a);
  }


  Quat exp(const Quat &a) {
    double len_v = length((Vec3&)a);
    double exp_w = std::exp(a.w);

    if (is_approx_zero(len_v)) return { exp_w * a, exp_w };

    return {
      (exp_w * std::sin(len_v) / len_v) * (Vec3&)a,
      exp_w * std::cos(len_v),
    };
  }


  Quat ln(const Quat &a) {
    double len = length(a);
    double len_v = length((Vec3&)a);

    if (is_approx_zero(len_v)) return { Vec3::ZERO, std::log(len) };
    
    return {
      (std::acos(a.w / len) / len_v) * (Vec3&)a,
      std::log(len),
    };
  }

  
  Quat pow(const Quat &a, const double b) {
    return exp(b * ln(a));
  }


  Quat slerp(const Quat &a, const Quat &b, const double t) {
    return a * pow(conjugate(a) * b, t);
  }


  // Dual-Quaternion specific functions


  DQuat dual_conjugate(const DQuat &a) {
    return {
      a.yz, a.zx, a.xy, a.w,
      -a.dx, -a.dy, -a.dz, -a.dxyz,
    };
  }


  DQuat quat_conjugate(const DQuat &a) {
    return {
      -a.yz, -a.zx, -a.xy, a.w,
      -a.dx, -a.dy, -a.dz, a.dxyz,
    };
  }

  DQuat dquat_conjugate(const DQuat &a) {
    return {
      -a.yz, -a.zx, -a.xy, a.w,
      a.dx, a.dy, a.dz, -a.dxyz,
    };
  }


  DQuat conjugate_unit(const DQuat &p, const DQuat &q) {
    return p * q * dquat_conjugate(p);
  }


  DQuat inverse(const DQuat &a) {
    const Quat &real = a.real_part();
    const Quat &dual = a.dual_part();
    Quat real_inv = inverse(real);
    return { real_inv, -dual * real_inv };
  }


  DQuat seplerp(const DQuat &a, const DQuat &b, const double t) {
    Vec3 a_trans = a.get_translation();
    Vec3 b_trans = b.get_translation();
    Quat a_rot = a.get_rotation();
    Quat b_rot = b.get_rotation();
    return DQuat::from_rotation_translation(slerp(a_rot, b_rot, t), lerp(a_trans, b_trans, t));
  }


  DQuat sclerp(const DQuat &a, const DQuat &b, const double t) {
    DQuat delta = quat_conjugate(a) * b;
    // Get delta's screw coordinates
    Vec3 direction, moment;
    double angle, translation;
    delta.get_screw_coordinates(direction, moment, angle, translation);
    // Calculate delta := delta^t - the delta to be used for the translation
    delta = DQuat::from_screw_coordinates(direction, moment, t * angle, t * translation);
    return a * delta;
  }


  DQuat kenlerp(const DQuat &a, const DQuat &b, const double t, const double beta) {
    DQuat sc_res = sclerp(a, b, t);
    Quat sc_rot = sc_res.get_rotation();
    Vec3 sc_trans = sc_res.get_translation();
    DQuat sep_res = seplerp(a, b, t);
    Quat sep_rot = sep_res.get_rotation();
    Vec3 sep_trans = sep_res.get_translation();
    return DQuat::from_rotation_translation(slerp(sc_rot, sep_rot, beta), lerp(sc_trans, sep_trans, beta));
  }

  // ==================
  // = Math operators =
  // ==================


  Vec3 operator+(const Vec3 &a, const Vec3 &b) {
    Vec3 r(a);
    r += b;
    return r;
  }


  Vec3 &operator+=(Vec3 &a, const Vec3 &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
  }


  Vec3 operator-(const Vec3 &a, const Vec3 &b) {
    Vec3 r(a);
    r -= b;
    return r;
  }


  Vec3 &operator-=(Vec3 &a, const Vec3 &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
  }


  Vec3 operator-(const Vec3 &a) {
    return { -a.x, -a.y, -a.z };
  }


  Vec3 operator*(const double a, const Vec3 &b) {
    Vec3 r(b);
    r *= a;
    return r;
  }


  Vec3 operator*(const Vec3 &a, const double b) {
    return b * a;
  }


  Vec3 &operator*=(Vec3 &a, const double b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    return a;
  }


  Vec3 operator/(const Vec3 &a, const double b) {
    Vec3 r(a);
    r /= b;
    return r;
  }


  Vec3 &operator/=(Vec3 &a, const double b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    return a;
  }


  Vec4 operator+(const Vec4 &a, const Vec4 &b) {
    Vec4 r(a);
    r += b;
    return r;
  }


  Vec4 &operator+=(Vec4 &a, const Vec4 &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
    return a;
  }


  Vec4 operator-(const Vec4 &a, const Vec4 &b) {
    Vec4 r(a);
    r -= b;
    return r;
  }


  Vec4 &operator-=(Vec4 &a, const Vec4 &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
    return a;    
  }


  Vec4 operator-(const Vec4 &a) {
    return { -a.x, -a.y, -a.z, -a.w };
  }


  Vec4 operator*(const double a, const Vec4 &b) {
    Vec4 r(b);
    r *= a;
    return r;
  }


  Vec4 operator*(const Vec4 &a, const double b) {
    return b * a;
  }


  Vec4 &operator*=(Vec4 &a, const double b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
    return a;
  }


  Vec4 operator/(const Vec4 &a, const double b) {
    Vec4 r(a);
    r /= b;
    return r;
  }


  Vec4 &operator/=(Vec4 &a, const double b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
    return a;
  }


  Quat operator+(const Quat &a, const Quat &b) {
    Quat r(a);
    r += b;
    return r;
  }


  Quat &operator+=(Quat &a, const Quat &b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
    return a;
  }


  Quat operator-(const Quat &a, const Quat &b) {
    Quat r(a);
    r -= b;
    return r;
  }


  Quat &operator-=(Quat &a, const Quat &b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
    return a;    
  }


  Quat operator-(const Quat &a) {
    return { -a.x, -a.y, -a.z, -a.w };
  }


  Quat operator*(const double a, const Quat &b) {
    Quat r(b);
    r *= a;
    return r;
  }


  Quat operator*(const Quat &a, const double b) {
    return b * a;
  }


  Quat &operator*=(Quat &a, const double b) {
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
    return a;
  }


  Quat operator/(const Quat &a, const double b) {
    Quat r(a);
    r /= b;
    return r;
  }


  Quat &operator/=(Quat &a, const double b) {
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
    return a;
  }


  Quat operator*(const Quat &a, const Quat &b) {
    return Quat {
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
      a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
      -a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w,
    };
  }


  Quat &operator*=(Quat &a, const Quat &b) {
    a = a * b;
    return a;
  }


  Vec3 operator*(const Mat3 &a, const Vec3 &b) {
    return Vec3 {
      a.x.x * b.x + a.y.x * b.y + a.z.x * b.z,
      a.x.y * b.x + a.y.y * b.y + a.z.y * b.z,
      a.x.z * b.x + a.y.z * b.y + a.z.z * b.z,
    };
  }


  Vec3 operator*(const Vec3 &a, const Mat3 &b) {
    return Vec3 {
      dot(a, b.x),
      dot(a, b.y),
      dot(a, b.z),
    };
  }


  Mat3 operator*(const Mat3 &a, const Mat3 &b) {
    return Mat3 {
      Vec3 {a * b.x},
      Vec3 {a * b.y},
      Vec3 {a * b.z},
    };
  }


  Mat3 &operator*=(Mat3 &a, const Mat3 &b) {
    a = a * b;
    return a;
  }


  Vec4 operator*(const Mat4 &a, const Vec4 &b) {
    return Vec4 {
      a.x.x * b.x + a.y.x * b.y + a.z.x * b.z + a.w.x * b.w,
      a.x.y * b.x + a.y.y * b.y + a.z.y * b.z + a.w.y * b.w,
      a.x.z * b.x + a.y.z * b.y + a.z.z * b.z + a.w.z * b.w,
      a.x.w * b.x + a.y.w * b.y + a.z.w * b.z + a.w.w * b.w,
    };
  }


  Vec4 operator*(const Vec4 &a, const Mat4 &b) {
    return Vec4 {
      dot(a, b.x),
      dot(a, b.y),
      dot(a, b.z),
      dot(a, b.w),
    };
  }


  Mat4 operator*(const Mat4 &a, const Mat4 &b) {
    return Mat4 {
      Vec4 {a * b.x},
      Vec4 {a * b.y},
      Vec4 {a * b.z},
      Vec4 {a * b.w},
    };
  }


  Mat4 &operator*=(Mat4 &a, const Mat4 &b) {
    a = a * b;
    return a;
  }


  DQuat operator+(const DQuat &a, const DQuat &b) {
    DQuat r(a);
    r += b;
    return r;
  }


  DQuat &operator+=(DQuat &a, const DQuat &b) {
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


  DQuat operator-(const DQuat &a, const DQuat &b) {
    DQuat r(a);
    r -= b;
    return r;
  }


  DQuat &operator-=(DQuat &a, const DQuat &b) {
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


  DQuat operator-(const DQuat &a) {
    return {
      -a.yz, -a.zx, -a.xy, -a.w,
      -a.dx, -a.dy, -a.dz, -a.dxyz,
    };
  }


  DQuat operator*(const double a, const DQuat &b) {
    DQuat r(b);
    r *= a;
    return r;
  }


  DQuat operator*(const DQuat &b, const double a) {
    return a * b;
  }


  DQuat &operator*=(DQuat &a, const double b) {
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


  DQuat operator/(const DQuat &a, const double b) {
    DQuat r(a);
    r /= b;
    return r;
  }


  DQuat &operator/=(DQuat &a, const double b) {
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


  DQuat operator*(const DQuat &a, const DQuat &b) {
    return {
      a.w * b.yz + a.yz * b.w + a.zx * b.xy - a.xy * b.zx,
      a.w * b.zx + a.zx * b.w + a.xy * b.yz - a.yz * b.xy,
      a.w * b.xy + a.xy * b.w + a.yz * b.zx - a.zx * b.yz,
      -a.yz * b.yz - a.zx * b.zx - a.xy * b.xy + a.w * b.w,

      a.dxyz * b.yz + a.dx * b.w + a.dy * b.xy - a.dz * b.zx + a.w * b.dx + a.yz * b.dxyz + a.zx * b.dz - a.xy * b.dy,
      a.dxyz * b.zx + a.dy * b.w + a.dz * b.yz - a.dx * b.xy + a.w * b.dy + a.zx * b.dxyz + a.xy * b.dx - a.yz * b.dz,
      a.dxyz * b.xy + a.dz * b.w + a.dx * b.zx - a.dy * b.yz + a.w * b.dz + a.xy * b.dxyz + a.yz * b.dy - a.zx * b.dx,
      -a.dx * b.yz - a.dy * b.zx - a.dz * b.xy + a.dxyz * b.w + -a.yz * b.dx - a.zx * b.dy - a.xy * b.dz + a.w * b.dxyz,
    };
  }


  DQuat &operator*=(DQuat &a, const DQuat &b) {
    a = a * b;
    return a;
  }


  // ===================
  // = Print operators =
  // ===================
  

  std::ostream &operator<<(std::ostream &os, const Vec3 &v) {
    os << "Vec3( " << v.x << ", " << v.y << ", " << v.z << " )";
    return os;
  }


  std::ostream &operator<<(std::ostream &os, const Vec4 &v) {
    os << "Vec4( " << v.x << ", " << v.y << ", " << v.z << ", " << v.w << " )";
    return os;
  }


  std::ostream &operator<<(std::ostream &os, const Quat &q) {
    os << q.x << " i + " << q.y << " j + " << q.z << " k + " << q.w;
    return os;
  }


  std::ostream &operator<<(std::ostream &os, const DQuat &q) {
    os << q.yz << " i + " << q.zx << " j + " << q.xy << " k + " << q.w << " + " << q.dx << " εi + " << q.dy << " εj + " << q.dz << " εk + " << q.dxyz << " ε";
    return os;
  }
}
