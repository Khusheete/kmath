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


#include "pga.hpp"
#include "kmath/constants.hpp"
#include "thirdparty/raylib/raylib.h"


using namespace kmath;


void draw_plane(const Vec3 &p_next_to, const Mvec3 &p_plane, const Color &p_color) {
  Mvec3 pos = Mvec3::point(p_next_to);
  pos = ((p_plane || pos) * p_plane).grade(3).point_normalize(); // Project point on the plane

  Mvec3 default_plane = Mvec3::plane(0.0, 1.0, 0.0, 0.0);
  Mvec3 rotation_axis = (p_plane.plane_normalize() & default_plane.plane_normalize()).grade(2);
  float rotation_angle = std::acos(rotation_axis.norm());
  rotation_axis = rotation_axis.line_normalize();

  Vector3 pos_vector = Vector3(
    pos[Mvec3::Basis::e032],
    pos[Mvec3::Basis::e013],
    pos[Mvec3::Basis::e021]
  );
  Vector3 rotation_vector = Vector3(
    rotation_axis[Mvec3::Basis::e23],
    rotation_axis[Mvec3::Basis::e31],
    rotation_axis[Mvec3::Basis::e12]
  );
  float scale = p_plane.norm();
  
  Mesh p = GenMeshPlane(1.0, 1.0, 1, 1);
  Model m = LoadModelFromMesh(p);
  
  DrawModelEx(
    m,
    pos_vector,
    rotation_vector,
    180.0 * rotation_angle / PI,
    Vector3(scale, scale, scale),
    p_color
  );

  UnloadModel(m);
  draw_line(p_next_to, rotation_axis, p_color);
}


void draw_line(const Vec3 &p_next_to, const Mvec3 &p_line, const Color &p_color) {
  Mvec3 pos = Mvec3::point(p_next_to);
  pos = ((pos || p_line) * p_line).grade(3).point_normalize(); // Project point on the line

  Mvec3 dir = -Mvec3::e0 * p_line;
  Mvec3 tip = (pos + dir).grade(3);

  DrawLine3D(
    Vector3(
      pos[Mvec3::Basis::e032],
      pos[Mvec3::Basis::e013],
      pos[Mvec3::Basis::e021]
    ),
    Vector3(
      tip[Mvec3::Basis::e032],
      tip[Mvec3::Basis::e013],
      tip[Mvec3::Basis::e021]
    ),
    p_color
  );
  DrawPoint3D(
    Vector3(
      pos[Mvec3::Basis::e032],
      pos[Mvec3::Basis::e013],
      pos[Mvec3::Basis::e021]
    ),
    p_color
  );
}

void draw_point(const Mvec3 &p_point, const Color &p_color) {
  Mvec3 normalized = p_point / p_point.norm();

  DrawPoint3D(
    Vector3(
      normalized[Mvec3::Basis::e032],
      normalized[Mvec3::Basis::e013],
      normalized[Mvec3::Basis::e021]
    ),
    p_color
  );
}
