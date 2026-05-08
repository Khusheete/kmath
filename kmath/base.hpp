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


#include "private/defines.hpp"
#include "concepts.hpp"

#include <concepts>
#include <cstddef>
#include <cstdlib>
#include <cmath>


namespace kmath {
  // ==========================
  // = Generalized operations =
  // ==========================

  template<typename R, typename A, Function<R, A> F>
  requires (!Indexable<A>)
  constexpr R apply(const A a, F op) {
    return op(a);
  }


  template<typename R, typename A, typename B, Function<R, A, B> F>
  requires (!Indexable<A>) && (!Indexable<B>)
  constexpr R apply(const A a, const B b, F op) {
    return op(a, b);
  }


  template<typename R, typename A, typename B, typename C, Function<R, A, B, C> F>
  requires (!Indexable<A>) && (!Indexable<B>) && (!Indexable<C>)
  constexpr R apply(const A a, const B b, const C c, F op) {
    return op(a, b, c);
  }


  template<typename R, typename A, typename B, typename C, typename D, Function<R, A, B, C, D> F>
  requires (!Indexable<A>) && (!Indexable<B>) && (!Indexable<C>)
  constexpr R apply(const A a, const B b, const C c, const D d, F op) {
    return op(a, b, c, d);
  }


  template<typename A, Function<A, A> F>
  requires (!Indexable<A>)
  constexpr A apply(const A a, F op) {
    return op(a);
  }


  template<typename A, Function<A, A, A> F>
  requires (!Indexable<A>)
  constexpr A apply(const A a, const A b, F op) {
    return op(a, b);
  }


  template<typename A, Function<A, A, A, A> F>
  requires (!Indexable<A>)
  constexpr A apply(const A a, const A b, const A c, F op) {
    return op(a, b, c);
  }


  template<typename A, Function<A, A, A, A, A> F>
  requires (!Indexable<A>)
  constexpr A apply(const A a, const A b, const A c, const A d, F op) {
    return op(a, b, c, d);
  }


  template<typename R, typename A, template<typename> typename AT, Function<R, A> F>
  requires ArrayTemplate<AT>
  constexpr AT<R> apply(const AT<A> a, F op) {
    AT<R> result{};
    for (size_t i = 0; i < AT<A>::SIZE; i++) {
      result[i] = op(a[i]);
    }
    return result;
  }


  template<typename R, typename A, typename B, template<typename> typename AT, Function<R, A, B> F>
  requires ArrayTemplate<AT>
  constexpr AT<R> apply(const AT<A> a, const AT<B> b, F op) {
    AT<R> result{};
    for (size_t i = 0; i < AT<A>::SIZE; i++) {
      result[i] = op(a[i], b[i]);
    }
    return result;
  }


  template<typename R, typename A, typename B, typename C, template<typename> typename AT, Function<R, A, B, C> F>
  requires ArrayTemplate<AT>
  constexpr AT<R> apply(const AT<A> a, const AT<B> b, const AT<C> c, F op) {
    AT<R> result{};
    for (size_t i = 0; i < AT<A>::SIZE; i++) {
      result[i] = op(a[i], b[i], c[i]);
    }
    return result;
  }


  template<typename R, typename A, typename B, typename C, typename D, template<typename> typename AT, Function<R, A, B, C, D> F>
  requires ArrayTemplate<AT>
  constexpr AT<R> apply(const AT<A> a, const AT<B> b, const AT<C> c, const AT<D> d, F op) {
    AT<R> result{};
    for (size_t i = 0; i < AT<A>::SIZE; i++) {
      result[i] = op(a[i], b[i], c[i], d[i]);
    }
    return result;
  }


  template<typename K, template<typename> typename AT, Function<K, K> F>
  requires ArrayTemplate<AT>
  constexpr AT<K> apply(const AT<K> a, F op) {
    AT<K> result{};
    for (size_t i = 0; i < AT<K>::SIZE; i++) {
      result[i] = op(a[i]);
    }
    return result;
  }


  template<typename K, template<typename> typename AT, Function<K, K, K> F>
  requires ArrayTemplate<AT>
  constexpr AT<K> apply(const AT<K> a, const AT<K> b, F op) {
    AT<K> result{};
    for (size_t i = 0; i < AT<K>::SIZE; i++) {
      result[i] = op(a[i], b[i]);
    }
    return result;
  }


  template<typename K, template<typename> typename AT, Function<K, K, K, K> F>
  requires ArrayTemplate<AT>
  constexpr AT<K> apply(const AT<K> a, const AT<K> b, const AT<K> c, F op) {
    AT<K> result{};
    for (size_t i = 0; i < AT<K>::SIZE; i++) {
      result[i] = op(a[i], b[i], c[i]);
    }
    return result;
  }


  template<typename K, template<typename> typename AT, Function<K, K, K, K, K> F>
  requires ArrayTemplate<AT>
  constexpr AT<K> apply(const AT<K> a, const AT<K> b, const AT<K> c, const AT<K> d, F op) {
    AT<K> result{};
    for (size_t i = 0; i < AT<K>::SIZE; i++) {
      result[i] = op(a[i], b[i], c[i], d[i]);
    }
    return result;
  }


  template<typename K>
  requires (!Indexable<K>)
  constexpr K select(const bool condition, const K a, const K b) {
    return condition ? a : b;
  }


  template<SizedIndexable V>
  constexpr V select(const BoolParameter<V> condition, const V a, const V b) {
    V result{};
    for (size_t i = 0; i < V::SIZE; i++) {
      result[i] = condition[i] ? a[i] : b[i];
    }
    return result;
  }


  // =====================
  // = Boolean Operators =
  // =====================

  template<Orderable K>
  requires (!Indexable<K>)
  constexpr bool equal(const K a, const K b) {
    return a == b;
  }


  template<Orderable K>
  requires (!Indexable<K>)
  constexpr bool not_equal(const K a, const K b) {
    return a == b;
  }


  template<Orderable K>
  requires (!Indexable<K>)
  constexpr bool less(const K a, const K b) {
    return a < b;
  }


  template<Orderable K>
  requires (!Indexable<K>)
  constexpr bool less_eq(const K a, const K b) {
    return a <= b;
  }


  template<Orderable K>
  requires (!Indexable<K>)
  constexpr bool greater(const K a, const K b) {
    return a > b;
  }


  template<Orderable K>
  requires (!Indexable<K>)
  constexpr bool greater_eq(const K a, const K b) {
    return a >= b;
  }


  template<Orderable K, template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr ST<bool> equal(const ST<K> a, const ST<K> b) {
    ST<bool> result{};
    for (size_t i = 0; i < ST<K>::SIZE; i++) {
      result[i] = equal(a[i], b[i]);
    }
    return result;
  }


  template<Orderable K, template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr ST<bool> not_equal(const ST<K> a, const ST<K> b) {
    ST<bool> result{};
    for (size_t i = 0; i < ST<K>::SIZE; i++) {
      result[i] = not_equal(a[i], b[i]);
    }
    return result;
  }


  template<Orderable K, template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr ST<bool> less(const ST<K> a, const ST<K> b) {
    ST<bool> result{};
    for (size_t i = 0; i < ST<K>::SIZE; i++) {
      result[i] = less(a[i], b[i]);
    }
    return result;
  }


  template<Orderable K, template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr ST<bool> less_eq(const ST<K> a, const ST<K> b) {
    ST<bool> result{};
    for (size_t i = 0; i < ST<K>::SIZE; i++) {
      result[i] = less_eq(a[i], b[i]);
    }
    return result;
  }


  template<Orderable K, template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr ST<bool> greater(const ST<K> a, const ST<K> b) {
    ST<bool> result{};
    for (size_t i = 0; i < ST<K>::SIZE; i++) {
      result[i] = greater(a[i], b[i]);
    }
    return result;
  }


  template<Orderable K, template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr ST<bool> greater_eq(const ST<K> a, const ST<K> b) {
    ST<bool> result{};
    for (size_t i = 0; i < ST<K>::SIZE; i++) {
      result[i] = greater_eq(a[i], b[i]);
    }
    return result;
  }


  constexpr bool all(const bool a) {
    return a;
  }


  template<template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr bool all(const ST<bool> a) {
    bool result = true;
    for (size_t i = 0; i < ST<bool>::SIZE; i++) {
      result &= a[i];
    }
    return result;
  }


  constexpr bool any(const bool a) {
    return a;
  }


  template<template<typename> typename ST>
  requires ArrayTemplate<ST>
  constexpr bool any(const ST<bool> a) {
    bool result = false;
    for (size_t i = 0; i < ST<bool>::SIZE; i++) {
      result |= a[i];
    }
    return result;
  }


  // ==========================
  // = Mathematical Functions =
  // ==========================


  template<Number V>
  constexpr V abs(const V a) {
    return select(greater_eq(a, V(0.0)), a, -a);
  }


  template<Number V>
  constexpr V sign(const V a) {
    return apply(a, [](const auto a) -> decltype(a) {
      using K = decltype(a);
      return K(less(K(0.0), a) - greater(K(0.0), a));
    });
  }


  template<Number V>
  constexpr V possign(const V a) {
    return apply(a, [](const auto a) -> decltype(a) {
      using K = decltype(a);
      return K(less_eq(K(0.0), a) - greater(K(0.0), a));
    });
  }


  template<Number V>
  constexpr V negsign(const V a) {
    return apply(a, [](const auto a) -> decltype(a) {
      using K = decltype(a);
      return K(less(K(0.0), a) - greater_eq(K(0.0), a));
    });
  }


  template<Number V>
  V mod(const V a, const V b) {
    return apply(a, b, [](const auto a, const auto b) {
      using K = decltype(a);
      if constexpr(is_integer<K>) {
        return a % b;
      } else if constexpr(is_floating_point<K>) {
        return std::fmod(a, b);
      } else {
        return mod(a, b);
      }
    });
  }


  template<Number V>
  V posmod(const V a, const V b) {
    return mod(mod(a, b) + b, b);
  }


  template<Number V>
  V remainder(const V a, const V b) {
    return apply(a, b, [](const auto a, const auto b) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::remainder(a, b);
      } else {
        return remainder(a, b);
      }
    });
  }


  template<Number V>
  V fma(const V a, const V b, const V c) {
    return apply(a, b, c, [](const auto a, const auto b, const auto c) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::fma(a, b, c);
      } else {
        return fma(a, b, c);
      }
    });
  }


  template<Number V>
  V max(const V a, const V b) {
    return apply(a, b, [](const auto a, const auto b) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::max(a, b);
      } else {
        return max(a, b);
      }
    });
  }


  template<Number V>
  V min(const V a, const V b) {
    return apply(a, b, [](const auto a, const auto b) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::min(a, b);
      } else {
        return min(a, b);
      }
    });
  }


  template<Number K>
  constexpr K lerp(const K a, const K b, const K t) {
    return t * (b - a) + a;
  }


  template<Number K, Vector<K> V>
  constexpr V lerp(const V a, const V b, const K t) {
    return t * (b - a) + a;
  }


  template<Number V>
  constexpr V inv_lerp(const V a, const V b, const V x) {
    return (x - a) / (b - a);
  }


  template<Number K, Vector<K> V>
  constexpr V inv_lerp(const K a, const K b, const V x) {
    return (x - V(a)) / (b - a);
  }


  template<Number K>
  constexpr K map(const K val, const K prev_min, const K prev_max, const K new_min, const K new_max) {
    return (val - prev_min) / (prev_max - prev_min) * (new_max - new_min) + new_min;
  }


  template<Number K, Vector<K> V>
  constexpr V map(const V val, const K prev_min, const K prev_max, const K new_min, const K new_max) {
    return (val - V(prev_min)) / ((prev_max - prev_min) * (new_max - new_min)) + V(new_min);
  }


  // ===================================
  // = Exponential and power functions =
  // ===================================


  template<Number V>
  inline V exp(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::exp(a);
      } else {
        return exp(a);
      }
    });
  }


  template<Number V>
  inline V exp2(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::exp2(a);
      } else {
        return exp2(a);
      }
    });
  }


  // Equivalent to exp(a) - V(1.0)
  template<Number V>
  inline V expm1(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::expm1(a);
      } else {
        return expm1(a);
      }
    });
  }


  template<Number V>
  inline V ln(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::log(a);
      } else if constexpr(requires{ln(a);}) {
        return ln(a);
      } else {
        return log(a);
      }
    });
  }


  template<Number V>
  inline V log10(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::log10(a);
      } else {
        return log10(a);
      }
    });
  }


  template<Number V>
  inline V log2(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::log2(a);
      } else {
        return log2(a);
      }
    });
  }


  // Equivalent to ln(a + V(1.0))
  template<Number V>
  inline V ln1p(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::log1p(a);
      } else {
        return log1p(a);
      }
    });
  }


  template<Number V>
  inline V pow(const V a, const V b) {
    return apply(a, b, [](const auto a, const auto b) -> decltype(a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::pow(a, b);
      } else {
        return pow(a, b);
      }
    });
  }


  template<Number K, SizedVector<K> V>
  inline V pow(const V a, const K b) {
    return apply(a, [&](const K a) -> decltype(a) {
      if constexpr(is_std_number<K>) {
        return std::pow(a, b);
      } else {
        return pow(a, b);
      }
    });
  }


  template<Number V, Integer I>
  requires (!std::same_as<V, I>)
  inline V pow(const V a, const I b) {
    return apply(a, [&](const auto a) -> decltype(a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::pow(a, b);
      } else {
        return pow(a, b);
      }
    });
  }


  template<Number K, Integer I, template<typename> typename VT>
  inline VT<K> pow(const VT<K> a, const VT<I> b) {
    return apply<K, I, K, VT>(a, b, [](const K a, const I b) -> K {
      if constexpr(is_std_number<K> && is_std_number<I>) {
        return std::pow(a, b);
      } else {
        return pow(a, b);
      }
    });
  }


  template<Number V>
  inline V sqrt(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::sqrt(a);
      } else {
        return sqrt(a);
      }
    });
  }


  template<Number V>
  inline V cbrt(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::cbrt(a);
      } else {
        return cbrt(a);
      }
    });
  }


  // ===========================
  // = Trigonometric functions =
  // ===========================


  template<Number V>
  inline V sin(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::sin(a);
      } else {
        return sin(a);
      }
    });
  }


  template<Number V>
  inline V cos(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::cos(a);
      } else {
        return cos(a);
      }
    });
  }


  template<Number V>
  inline V tan(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::tan(a);
      } else {
        return tan(a);
      }
    });
  }


  template<Number V>
  inline V asin(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::asin(a);
      } else {
        return asin(a);
      }
    });
  }


  template<Number V>
  inline V acos(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::acos(a);
      } else {
        return acos(a);
      }
    });
  }


  template<Number V>
  inline V atan(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::atan(a);
      } else {
        return atan(a);
      }
    });
  }


  template<Number V>
  inline V atan2(const V y, const V x) {
    return apply(y, x, [](const auto y, const auto x) {
      if constexpr(is_std_number<decltype(y)>) {
        return std::atan2(y, x);
      } else {
        return atan2(y, x);
      }
    });
  }


  // ========================
  // = Hyperbolic functions =
  // ========================


  template<Number V>
  inline V sinh(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::sinh(a);
      } else {
        return sinh(a);
      }
    });
  }


  template<Number V>
  inline V cosh(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::cosh(a);
      } else {
        return cosh(a);
      }
    });
  }


  template<Number V>
  inline V tanh(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::tanh(a);
      } else {
        return tanh(a);
      }
    });
  }


  template<Number V>
  inline V asinh(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::asinh(a);
      } else {
        return asinh(a);
      }
    });
  }


  template<Number V>
  inline V acosh(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::acosh(a);
      } else {
        return acosh(a);
      }
    });
  }


  template<Number V>
  inline V atanh(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::atanh(a);
      } else {
        return atanh(a);
      }
    });
  }


  // =============================
  // = Gamma and error functions =
  // =============================


  // Error function
  template<Number V>
  inline V erf(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::erf(a);
      } else {
        return erf(a);
      }
    });
  }


  // Complementary error function
  template<Number V>
  inline V erfc(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::erfc(a);
      } else {
        return erfc(a);
      }
    });
  }


  // Gamma function
  template<Number V>
  inline V tgamma(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::tgamma(a);
      } else {
        return tgamma(a);
      }
    });
  }


  // Natural log of the gamma function
  template<Number V>
  inline V lngamma(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::lgamma(a);
      } else if constexpr(requires{lgamma(a);}) {
        return lgamma(a);
      } else {
        return lngamma(a);
      }
    });
  }


  // ======================
  // = Rounding Functions =
  // ======================


  template<Number V>
  inline V ceil(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::ceil(a);
      } else {
        return ceil(a);
      }
    });
  }


  template<Number V>
  inline V floor(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::floor(a);
      } else {
        return floor(a);
      }
    });
  }


  template<Number V>
  inline V trunc(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::trunc(a);
      } else {
        return trunc(a);
      }
    });
  }


  template<Number V>
  inline V round(const V a) {
    return apply(a, [](const auto a) {
      if constexpr(is_std_number<decltype(a)>) {
        return std::round(a);
      } else {
        return round(a);
      }
    });
  }


  // =================================
  // = Floating-point classification =
  // =================================


  template<Number V>
  inline BoolParameter<V> is_finite(const V a) {
    return apply<bool>(a, [](const auto a) {
       if constexpr(is_std_number<decltype(a)>) {
         return std::isfinite(a);
       } else {
         return is_finite(a);
       }
    });
  }


  template<Number V>
  inline BoolParameter<V> is_infinite(const V a) {
    return apply<bool>(a, [](const auto a) {
       if constexpr(is_std_number<decltype(a)>) {
         return std::isinf(a);
       } else {
         return is_infinite(a);
       }
    });
  }


  template<Number V>
  inline BoolParameter<V> is_nan(const V a) {
    return apply<bool>(a, [](const auto a) {
       if constexpr(is_std_number<decltype(a)>) {
         return std::isnan(a);
       } else {
         return is_nan(a);
       }
    });
  }


  template<Number V>
  inline BoolParameter<V> is_normal_number(const V a) {
    return apply<bool>(a, [](const auto a) {
       if constexpr(is_std_number<decltype(a)>) {
         return std::isnormal(a);
       } else {
         return is_normal_number(a);
       }
    });
  }


  template<Number V>
  inline BoolParameter<V> sign_bit(const V a) {
    return apply<bool>(a, [](const auto a) {
       if constexpr(is_std_number<decltype(a)>) {
         return std::signbit(a);
       } else {
         return sign_bit(a);
       }
    });
  }


  // ========================
  // = Approximate Equality =
  // ========================


  template<Number V>
  constexpr BoolParameter<V> approx_zero(const V a) {
    return less_eq(abs(a), V(KMATH_EPSILON));
  }


  template<Number V>
  constexpr bool is_approx_zero(const V a) {
    return all(approx_zero(a));
  }


  template<typename V>
  constexpr BoolParameter<V> approx(const V a, const V b) {
    return approx_zero(b - a);
  }


  template<typename V>
  constexpr bool is_approx(const V a, const V b) {
    return is_approx_zero(b - a);
  }
}
