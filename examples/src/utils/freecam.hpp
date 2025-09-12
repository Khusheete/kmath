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


#include "kmath/kmath.hpp"
#include "thirdparty/raylib/raylib.h"


struct FreeCam {
  Camera3D rl_camera;
  kmath::Vec3 position;
  kmath::Rotor3 direction;

  FreeCam()
    : rl_camera({
      .position = Vector3(),
      .target = Vector3(0.0, 0.0, -1.0),
      .up = Vector3(0.0, 1.0, 0.0),
      .fovy = 70,
      .projection = CAMERA_PERSPECTIVE,
    }),
    position(kmath::Vec3::ZERO),
    direction(kmath::Rotor3::IDENTITY)
  {}

  void update(float p_delta);
  void begin_mode_3d() const;
};
