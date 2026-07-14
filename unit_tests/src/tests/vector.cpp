#include "vector.hpp"
#include "../testing.hpp"

#include "kmath/vector.hpp"


using namespace kmath;


void test_vector2() {
  const _Vec2<float> a(1.0, 2.0);
  const _Vec2<float> b(-2.0, 5.0);

  UNIT_TEST("Length", {
    TEST_EQ_APPROX("||a||^2", length_squared(a), 5.0f);
    TEST_EQ_APPROX("||a||", length(a), (float)std::sqrt(5.0));
    TEST_EQ_APPROX("||b||^2", length_squared(b), 29.0f);
    TEST_EQ_APPROX("||b||", length(b), (float)std::sqrt(29.0));
  });
  UNIT_TEST("Normalized", {
    TEST_EQ_APPROX("normalize b", normalized(b), _Vec2<float>(-2.0 / std::sqrt(29.0), 5.0 / std::sqrt(29.0)));
  });
  UNIT_TEST("Dot product", {
    TEST_EQ_APPROX("a . b", dot(a, b), 8.0f);
  });
  UNIT_TEST("Addition", {
    _Vec2<float> c(a);
    TEST_EQ_APPROX("a + b", a + b, _Vec2<float>(-1.0, 7.0));
    TEST_EQ_APPROX("c += b", c += b, _Vec2<float>(-1.0, 7.0));
  });
  UNIT_TEST("Subtraction", {
    _Vec2<float> c(a);
    TEST_EQ_APPROX("a - b", a - b, _Vec2<float>(3.0, -3.0));
    TEST_EQ_APPROX("c -= b", c -= b, _Vec2<float>(3.0, -3.0));
  });
  UNIT_TEST("Negation", {
    TEST_EQ_APPROX("-b", -b, _Vec2<float>(2.0, -5.0));
  });
  UNIT_TEST("Scalar multiplication", {
    _Vec2<float> c(a);
    TEST_EQ_APPROX("2.0 * a", 2.0f * a, _Vec2<float>(2.0, 4.0));
    TEST_EQ_APPROX("b * 2.0", b * 2.0f, _Vec2<float>(-4.0, 10.0));
    TEST_EQ_APPROX("-3.0 * a", -3.0f * a, _Vec2<float>(-3.0, -6.0));
    TEST_EQ_APPROX("b * -3.0", b * -3.0f, _Vec2<float>(6.0, -15.0));
    TEST_EQ_APPROX("c *= 1.5", c *= 1.5f, _Vec2<float>(1.5, 3.0));
  });
  UNIT_TEST("Scalar division", {
    _Vec2<float> c(a);
    TEST_EQ_APPROX("b / 2.0", b / 2.0f, _Vec2<float>(-1.0, 2.5));
    TEST_EQ_APPROX("b / -0.25", b / -0.25f, _Vec2<float>(8.0, -20.0));
    TEST_EQ_APPROX("c /= 0.5", c /= 0.5f, _Vec2<float>(2.0, 4.0));
  });
}


void test_vector3() {
  const _Vec3<float> a(1.0, 2.0, 3.0);
  const _Vec3<float> b(-2.0, 5.0, 1.0);

  UNIT_TEST("Length", {
    TEST_EQ_APPROX("||a||^2", length_squared(a), 14.0f);
    TEST_EQ_APPROX("||a||", length(a), (float)std::sqrt(14.0));
    TEST_EQ_APPROX("||b||^2", length_squared(b), 30.0f);
    TEST_EQ_APPROX("||b||", length(b), (float)std::sqrt(30.0));
  });
  UNIT_TEST("Normalized", {
    TEST_EQ_APPROX("normalize b", normalized(b), _Vec3<float>(-2.0 / std::sqrt(30.0), 5.0 / std::sqrt(30.0), 1.0 / std::sqrt(30.0)));
  });
  UNIT_TEST("Dot product", {
    TEST_EQ_APPROX("a . b", dot(a, b), 11.0f);
  });
  UNIT_TEST("Cross product", {
    TEST_EQ_APPROX("a x b", cross(a, b), _Vec3<float>(-13.0, -7.0, 9.0));
  });
  UNIT_TEST("Addition", {
    _Vec3<float> c(a);
    TEST_EQ_APPROX("a + b", a + b, _Vec3<float>(-1.0, 7.0, 4.0));
    TEST_EQ_APPROX("c += b", c += b, _Vec3<float>(-1.0, 7.0, 4.0));
  });
  UNIT_TEST("Subtraction", {
    _Vec3<float> c(a);
    TEST_EQ_APPROX("a - b", a - b, _Vec3<float>(3.0, -3.0, 2.0));
    TEST_EQ_APPROX("c -= b", c -= b, _Vec3<float>(3.0, -3.0, 2.0));
  });
  UNIT_TEST("Negation", {
    TEST_EQ_APPROX("-b", -b, _Vec3<float>(2.0, -5.0, -1.0));
  });
  UNIT_TEST("Scalar multiplication", {
    _Vec3<float> c(a);
    TEST_EQ_APPROX("2.0 * a", 2.0f * a, _Vec3<float>(2.0, 4.0, 6.0));
    TEST_EQ_APPROX("b * 2.0", b * 2.0f, _Vec3<float>(-4.0, 10.0, 2.0));
    TEST_EQ_APPROX("-3.0 * a", -3.0f * a, _Vec3<float>(-3.0, -6.0, -9.0));
    TEST_EQ_APPROX("b * -3.0", b * -3.0f, _Vec3<float>(6.0, -15.0, -3.0));
    TEST_EQ_APPROX("c *= 1.5", c *= 1.5f, _Vec3<float>(1.5, 3.0, 4.5));
  });
  UNIT_TEST("Scalar division", {
    _Vec3<float> c(a);
    TEST_EQ_APPROX("b / 2.0", b / 2.0f, _Vec3<float>(-1.0, 2.5, 0.5));
    TEST_EQ_APPROX("b / -0.25", b / -0.25f, _Vec3<float>(8.0, -20.0, -4.0));
    TEST_EQ_APPROX("c /= 0.5", c /= 0.5f, _Vec3<float>(2.0, 4.0, 6.0));
  });
}


void test_vector4() {
  const _Vec4<float> a(1.0, 2.0, 3.0, -1.0);
  const _Vec4<float> b(-2.0, 5.0, 1.0, 2.0);

  UNIT_TEST("Length", {
    TEST_EQ_APPROX("||a||^2", length_squared(a), 15.0f);
    TEST_EQ_APPROX("||a||", length(a), (float)std::sqrt(15.0));
    TEST_EQ_APPROX("||b||^2", length_squared(b), 34.0f);
    TEST_EQ_APPROX("||b||", length(b), (float)std::sqrt(34.0));
  });
  UNIT_TEST("Normalized", {
    TEST_EQ_APPROX("normalize b", normalized(b), _Vec4<float>(-2.0 / std::sqrt(34.0), 5.0 / std::sqrt(34.0), 1.0 / std::sqrt(34.0), 2.0 / std::sqrt(34.0)));
  });
  UNIT_TEST("Dot product", {
    TEST_EQ_APPROX("a . b", dot(a, b), 9.0f);
  });
  UNIT_TEST("Addition", {
    _Vec4<float> c(a);
    TEST_EQ_APPROX("a + b", a + b, _Vec4<float>(-1.0, 7.0, 4.0, 1.0));
    TEST_EQ_APPROX("c += b", c += b, _Vec4<float>(-1.0, 7.0, 4.0, 1.0));
  });
  UNIT_TEST("Subtraction", {
    _Vec4<float> c(a);
    TEST_EQ_APPROX("a - b", a - b, _Vec4<float>(3.0, -3.0, 2.0, -3.0));
    TEST_EQ_APPROX("c -= b", c -= b, _Vec4<float>(3.0, -3.0, 2.0, -3.0));
  });
  UNIT_TEST("Negation", {
    TEST_EQ_APPROX("-b", -b, _Vec4<float>(2.0, -5.0, -1.0, -2.0));
  });
  UNIT_TEST("Scalar multiplication", {
    _Vec4<float> c(a);
    TEST_EQ_APPROX("2.0 * a", 2.0f * a, _Vec4<float>(2.0, 4.0, 6.0, -2.0));
    TEST_EQ_APPROX("b * 2.0", b * 2.0f, _Vec4<float>(-4.0, 10.0, 2.0, 4.0));
    TEST_EQ_APPROX("-3.0 * a", -3.0f * a, _Vec4<float>(-3.0, -6.0, -9.0, 3.0));
    TEST_EQ_APPROX("b * -3.0", b * -3.0f, _Vec4<float>(6.0, -15.0, -3.0, -6.0));
    TEST_EQ_APPROX("c *= 1.5", c *= 1.5f, _Vec4<float>(1.5, 3.0, 4.5, -1.5));
  });
  UNIT_TEST("Scalar division", {
    _Vec4<float> c(a);
    TEST_EQ_APPROX("b / 2.0", b / 2.0f, _Vec4<float>(-1.0, 2.5, 0.5, 1.0));
    TEST_EQ_APPROX("b / -0.25", b / -0.25f, _Vec4<float>(8.0, -20.0, -4.0, -8.0));
    TEST_EQ_APPROX("c /= 0.5", c /= 0.5f, _Vec4<float>(2.0, 4.0, 6.0, -2.0));
  });
}
