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


using namespace kmath;


void draw_plane(const Vec3f &p_next_to, const Mvec3df &p_plane, const Color &p_color) {
  Mvec3df pos = Mvec3df::point(p_next_to);
  pos = (pos || p_plane) * p_plane; // Project point on the plane

  // TODO: implement
}


void draw_line(const Vec3f &p_next_to, const Mvec3df &p_line, const Color &p_color) {
  Mvec3df pos = Mvec3df::point(p_next_to);
  pos = ((pos || p_line) * p_line).grade(3).point_normalize(); // Project point on the line

  Mvec3df dir = -Mvec3df::e0 * p_line;
  Mvec3df tip = (pos + dir).grade(3);

  DrawLine3D(
    Vector3(
      pos[Mvec3df::Basis::e032],
      pos[Mvec3df::Basis::e013],
      pos[Mvec3df::Basis::e021]
    ),
    Vector3(
      tip[Mvec3df::Basis::e032],
      tip[Mvec3df::Basis::e013],
      tip[Mvec3df::Basis::e021]
    ),
    p_color
  );
  DrawPoint3D(
    Vector3(
      pos[Mvec3df::Basis::e032],
      pos[Mvec3df::Basis::e013],
      pos[Mvec3df::Basis::e021]
    ),
    p_color
  );
}

void draw_point(const Mvec3df &p_point, const Color &p_color) {
  Mvec3df normalized = p_point / p_point.norm();

  DrawPoint3D(
    Vector3(
      normalized[Mvec3df::Basis::e032],
      normalized[Mvec3df::Basis::e013],
      normalized[Mvec3df::Basis::e021]
    ),
    p_color
  );
}
