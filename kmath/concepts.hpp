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


#include <complex>
#include <concepts>
#include <type_traits>


namespace kmath {

  // =========================
  // = Mathematical concepts =
  // =========================

  template<typename N>
  concept Number = requires(N n1, N n2) {
    n1 += n2;
    n1 -= n2;
    n1 *= n2;
    n1 /= n2;
    n1 + n2;
    n1 - n2;
    n1 * n2;
    n1 / n2;
    N(0.0);
    N(0);
  };


  template<typename S>
  concept Orderable = requires(S a, S b) {
    { a == b } -> std::same_as<bool>;
    { a < b }  -> std::same_as<bool>;
    { a > b }  -> std::same_as<bool>;
    { a >= b } -> std::same_as<bool>;
    { a <= b } -> std::same_as<bool>;
  };


  template<typename V, typename K>
  concept Vector = requires(V v1, V v2, K l) {
    v1 += v2;
    v1 -= v2;
    v1 *= l;
    v1 /= l;
    v1 + v2;
    v1 - v2;
    l * v1;
    v1 * l;
    v1 / l;
  } && Number<K> && (!std::same_as<V, K>);


  template<template<typename> typename VT>
  concept VectorTemplate = Vector<VT<float>, float>;


  template<typename V>
  concept Normed = requires(V v) {
    length(v);
    length_squared(v);
  };


  template<typename V, typename K>
  concept NormedVector = Vector<V, K> && Normed<V>;


  // =======================
  // = Programing concepts =
  // =======================


  template<typename T>
  concept Indexable = requires(T t, std::size_t index) {
    t[index];
  };


  template<typename T>
  concept SizedIndexable = requires(T t) {
    { T::SIZE } -> std::same_as<const std::size_t &>;
  } && Indexable<T>;


  template<typename T, typename R>
  concept Array = requires(T t, R r, std::size_t index) {
    r = t[index];
  } && SizedIndexable<T>;


  template<template<typename> typename ST>
  concept ArrayTemplate = Array<ST<int>, int>;


  template<typename V, typename K>
  concept SizedVector = Vector<V, K> && SizedIndexable<V>;


  template<template<typename> typename VT>
  concept SizedVectorTemplate = SizedVector<VT<float>, float>;


  template<typename A, typename B>
  concept Castable = requires(A a) {
    B(a);
  };


  template<typename F, typename R, typename ...Ts>
  concept Function = requires(F f, Ts ...ts) {
    { f(ts...) } -> std::same_as<R>;
  };


  // =============================
  // = Standard numbers concepts =
  // =============================


  template<typename T>
  concept FloatingPoint = std::floating_point<T>;


  template<typename T>
  constexpr const bool is_floating_point = std::is_floating_point_v<T>;


  template<typename T>
  concept Integer = std::integral<T>;


  template<typename T>
  constexpr const bool is_integer = std::is_integral_v<T>;


  template<typename T>
  struct _is_complex {
    static constexpr const bool value = false;
  };


  template<typename T>
  struct _is_complex<std::complex<T>> {
    static constexpr const bool value = true;
  };


  template<typename T>
  concept Complex = _is_complex<T>::value;


  template<typename T>
  constexpr const bool is_complex = _is_complex<T>::value;


  template<typename T>
  concept StdNumber = Integer<T> || FloatingPoint<T> || Complex<T>;


  template<typename T>
  constexpr const bool is_std_number = StdNumber<T>;


  // ========================
  // = Type transformations =
  // ========================


  template<typename T, typename S>
  struct _parameter_swap {
    using type = S;
  };


  template<template<typename> typename ST, typename K, typename S>
  struct _parameter_swap<ST<K>, S> {
    using type = ST<S>;
  };


  template<typename T>
  using _bool_parameter = _parameter_swap<T, bool>;


  template<typename T>
  struct _parameter {
    using type = T;
  };


  template<template<typename> typename ST, typename T>
  struct _parameter<ST<T>> {
    using type = T;
  };


  // Swaps the parameter of a templated type T to S. If T is not a templated type, the new type is S.
  template<typename T, typename S>
  using ParameterSwap = _parameter_swap<T, S>::type;


  // Equivalent to ParameterSwap<T, bool>.
  template<typename T>
  using BoolParameter = _bool_parameter<T>::type;


  // Gives the type of the parameter of the templated type T. If T is not templated returns T.
  template<typename T>
  using Parameter = _parameter<T>::type;
}
