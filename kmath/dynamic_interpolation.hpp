// Copyright © 2026 Souchet Ferdinand (aka. Khusheete)
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


#ifndef KMATH_DYNAMIC_INTERPOLATION_HPP
#define KMATH_DYNAMIC_INTERPOLATION_HPP


#include "base.hpp"
#include "concepts.hpp"


namespace kmath {

  template<Number T>
  class ExponentialInterpolator {
  public:
    template<AnyVector<T> V>
    void interpolate(V &position, const V &target, const T &delta) const {
      position = lerp(position, target, T(1) - exp(-delta * inv_characteristic_time));
    }


    void set_parameters(const T &characteristic_time) {
      inv_characteristic_time = T(1) / characteristic_time;
    }


    void set_characteristic_time(const T &characteristic_time) {
      set_parameters(characteristic_time);
    }


    T get_characteristic_time() const {
      return T(1) / inv_characteristic_time;
    }
    

    ExponentialInterpolator(const T &characteristic_time)
    : inv_characteristic_time(T(1) / characteristic_time)
    {}
    
  private:
    T inv_characteristic_time;
  };


  template<Number T>
  class SpringInterpolator {
  public:
    enum class Model {
      APERIODIC,
      CRITICAL,
      PSEUDO_PERIODIC,
    };

  public:
    template<AnyVector<T> V>
    void interpolate(V &position, V &velocity, const V &target, const T &delta) const {
      switch (model) {
      break;case Model::APERIODIC:
        _interpolate_aperiodic(position, velocity, target, delta);
      break;case Model::CRITICAL:
        _interpolate_critical(position, velocity, target, delta);
      break;case Model::PSEUDO_PERIODIC:
        _interpolate_pseudo_periodic(position, velocity, target, delta);
      }
    }

    
    void set_parameters(const T &stiffness, const T &drag) {
      this->stiffness = max(stiffness, T(0));
      this->drag = max(drag, T(0));

      const T omega_0 = sqrt(stiffness);
      const T inv_drag = T(1) / drag;
      const T quality = omega_0 * inv_drag;

      if (is_approx(quality, T(0.5))) {
        model = Model::CRITICAL;
        alpha = T(2) * inv_drag; // Characteristic time
      } else if (quality < T(0.5)) {
        model = Model::APERIODIC;
        alpha = sqrt(T(1) - T(4) * quality * quality); // Root delta
        beta = -T(1) * inv_drag / alpha; // Initial condition matrix determinant
      } else {
        model = Model::PSEUDO_PERIODIC;
        alpha = drag * sqrt(quality * quality - T(0.25)); // Pseudo-pulse
        beta = T(0.5) * drag; // Inverse of characteristic time
      }
    }


    void set_stiffness(const T &stiffness) {
      set_parameters(stiffness, drag);
    }


    void set_drag(const T &drag) {
      set_parameters(stiffness, drag);
    }


    T get_stiffness() const {
      return stiffness;
    }


    T get_drag() const {
      return drag;
    }


    Model get_model() const {
      return model;
    }


    SpringInterpolator(const T &stiffness, const T &drag) {
      set_parameters(stiffness, drag);
    }


  private:
    template<AnyVector<T> V>
    void _interpolate_aperiodic(V &position, V &velocity, const V &target, const T &delta) const {
      // Characteristic polynomial roots
      const T lambda_1 = -drag * T(0.5) * (T(1) - alpha);
      const T lambda_2 = -drag * T(0.5) * (T(1) + alpha);

      // Coordinates of the specific solution in the vector space of solutions
      const V target_delta = position - target;
      const V a = beta * (lambda_2 * target_delta - velocity);
      const V b = beta * (-lambda_1 * target_delta + velocity);

      // Interpolation step
      const V part_1 = a * exp(lambda_1 * delta);
      const V part_2 = b * exp(lambda_2 * delta);
      position = part_1 + part_2 + target;
      velocity = lambda_1 * part_1 + lambda_2 * part_2;
    }


    template<AnyVector<T> V>
    void _interpolate_critical(V &position, V &velocity, const V &target, const T &delta) const {
      const T inv_alpha = T(1) / alpha;
      
      // Coordinates of the specific solution in the vector space of solutions
      const V a = position - target;
      const V b = velocity + a * inv_alpha;

      // Interpolation step
      const T exp_ttau = exp(-delta * inv_alpha);
      position = (a + b * delta) * exp_ttau + target;
      velocity = (-a * inv_alpha + (T(1) - delta * inv_alpha) * b) * exp_ttau;
    }


    template<AnyVector<T> V>
    void _interpolate_pseudo_periodic(V &position, V &velocity, const V &target, const T &delta) const {
      const T inv_alpha = T(1) / alpha;

      // Coordinates of the specific solution in the vector space of solutions
      const V a = position - target;
      const V b = (velocity + a * beta) * inv_alpha;

      // Interpolation step
      const T cos_wdt = cos(alpha * delta);
      const T sin_wdt = sin(alpha * delta);
      const T exp_ttau = exp(-delta * beta);
      position = (a * cos_wdt + b * sin_wdt) * exp_ttau + target;
      velocity = ((b * alpha - a * beta) * cos_wdt - (a * alpha + b * beta) * sin_wdt) * exp_ttau;
    }


  private:
    // Parameters
    T stiffness, drag;
    // Precalculated values
    T alpha, beta;
    Model model;
  };
}


#endif
