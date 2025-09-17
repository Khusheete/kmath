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


#include "color.hpp"
#include "vector.hpp"
#include <cmath>


namespace kmath {
  // ==============================
  // = Functions for calculations =
  // ==============================
  
  
  typedef _Vec2<float> LC;
  typedef _Vec2<float> ST;


  inline float toe(const float x) {
    constexpr float k1 = 0.206f;
    constexpr float k2 = 0.03f;
    constexpr float k3 = (1.0f + k1) / (1.0f + k2);
    return 0.5f * (k3 * x - k1 + sqrtf((k3 * x - k1) * (k3 * x - k1) + 4 * k2 * k3 * x));
  }


  inline float toe_inv(const float x) {
    constexpr float k1 = 0.206f;
    constexpr float k2 = 0.03f;
    constexpr float k3 = (1.0f + k1) / (1.0f + k2);
    return (x * x + k1 * x) / (k3 * (x + k2));
  }


  inline ST to_ST(const LC &cusp) {
    float L = cusp.x;
    float C = cusp.y;
    return ST(C / L, C / (1.0f - L));
  }


  // ===================
  // = OkLab functions =
  // ===================


  OkLab xyz_to_oklab(const XYZD65 &xyz) {
    const Mat3 M1(
      Vec3(+0.8189330101f, +0.0329845436f, +0.0482003018f),
      Vec3(+0.3618667424f, +0.9293118715f, +0.2643662691f),
      Vec3(-0.1288597137f, +0.0361456387f, +0.6338517070f)
    );
    const Mat3 M2(
      Vec3(+0.2104542553f, +1.9779984951f, +0.0259040371f),
      Vec3(+0.7936177850f, -2.4285922050f, +0.7827717662f),
      Vec3(-0.0040720468f, +0.4505937099f, -0.8086757660f)
    );
    Vec3 lms = M1 * xyz;
    lms = Vec3(
      std::cbrt(lms.x),
      std::cbrt(lms.y),
      std::cbrt(lms.z)
    );
    return M2 * lms;
  }


  XYZD65 oklab_to_xyz(const OkLab &lab) {
    const Mat3 M2(
      Vec3(+0.9999999984505197f , +1.0000000088817607f , +1.0000000546724108f),
      Vec3(+0.39633779217376786f, -0.10556134232365633f, -0.08948418209496574f),
      Vec3(+0.21580375806075877f, -0.06385417477170588f, -1.2914855378640917f)
    );
    const Mat3 M1(
      Vec3(+1.2270138511035211f, -0.0405801784232806f, -0.0763812845057069f),
      Vec3(-0.5577999806518222f, +1.11225686961683f  , -0.4214819784180127f),
      Vec3(+0.2812561489664678f, -0.0716766786656012f, +1.5861632204407947f)
    );
    Vec3 lms = M2 * lab;
    lms = lms * lms * lms;
    return M1 * lms;
  }
  

  Lsrgb oklab_to_lrgb(const OkLab &lab) {
    const Mat3 M2(
      Vec3(+1.0000000000f, +1.0000000000f, +1.0000000000f),
      Vec3(+0.3963377774f, -0.1055613458f, -0.0894841775f),
      Vec3(+0.2158037573f, -0.0638541728f, -1.2914855480f)
    );
    const Mat3 M1(
      Vec3(+4.0767416621f, -1.2684380046f, -0.0041960863f),
      Vec3(-3.3077115913f, +2.6097574011f, -0.7034186147f),
      Vec3(+0.2309699292f, -0.3413193965f, +1.7076147010f)
    );
    Vec3 lms = M2 * lab;
    lms = lms * lms * lms;
    return M1 * lms;
  }


  OkLab lrgb_to_oklab(const Lsrgb &rgb) {
    const Mat3 M1(
      Vec3(+0.4122214708f, +0.2119034982f, +0.0883024619f),
      Vec3(+0.5363325363f, +0.6806995451f, +0.2817188376f),
      Vec3(+0.0514459929f, +0.1073969566f, +0.6299787005f)
    );
    const Mat3 M2(
      Vec3(+0.2104542553f, +1.9779984951f, +0.0259040371f),
      Vec3(+0.7936177850f, -2.4285922050f, +0.7827717662f),
      Vec3(-0.0040720468f, +0.4505937099f, -0.8086757660f)
    );
    Vec3 lms = M1 * rgb;
    lms = Vec3(
      std::cbrt(lms.x),
      std::cbrt(lms.y),
      std::cbrt(lms.z)
    );
    return M2 * lms;
  }

}
