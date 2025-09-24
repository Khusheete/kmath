#include "kmath/color.hpp"
#include "testing.hpp"

#include "kmath/color.hpp"
#include "kmath/euclidian_flat_3d.hpp"
#include "kmath/interpolation_functions.hpp"
#include "kmath/matrix.hpp"
#include "kmath/motor_3d.hpp"
#include "kmath/rotor_3d.hpp"
#include "kmath/vector.hpp"
#include "kmath/utils.hpp"

#include <cmath>
#include <cstdlib>


using namespace kmath;


int main(void) {
  Testing::init_singleton();

  // TODO: test operations between flats
  // TODO: test projections
  // TODO: test rejections
  // TODO: test reflections

  UNIT_TEST_SECTION("Vector2", {
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
  })


  UNIT_TEST_SECTION("Vector3", {
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
  })


  UNIT_TEST_SECTION("Vector4", {
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
  })


  UNIT_TEST_SECTION("Plane3", {
    const Plane3 a = Plane3::plane(Vec3(1.0, -2.0, 3.0), 5.0);
    const Plane3 b = Plane3::plane(Vec3::ZERO, -2.0);
    const Plane3 c = Plane3::plane(Vec3(4.0, 2.0, -1.0), -2.0);
    const Plane3 d = Plane3::plane(Vec3(0.0, 1.0, 0.0), 12.0);

    UNIT_TEST("Addition", {
      Plane3 e(a);
      TEST_EQ_APPROX("a + c", a + c, Plane3::plane(Vec3(5.0, 0.0, 2.0), 3.0));
      TEST_EQ_APPROX("e += c", e += c, Plane3::plane(Vec3(5.0, 0.0, 2.0), 3.0));
    });
    UNIT_TEST("Subtraction", {
      Plane3 e(a);
      TEST_EQ_APPROX("a - c", a - c, Plane3::plane(Vec3(-3.0, -4.0, 4.0), 7.0));
      TEST_EQ_APPROX("e -= c", e -= c, Plane3::plane(Vec3(-3.0, -4.0, 4.0), 7.0));
    });
    UNIT_TEST("Scalar multiplication", {
      Plane3 e(a);
      TEST_EQ_APPROX("a * 2.0", a * 2.0f, Plane3::plane(Vec3(2.0, -4.0, 6.0), 10.0));
      TEST_EQ_APPROX("2.0 * a", 2.0f * a, Plane3::plane(Vec3(2.0, -4.0, 6.0), 10.0));
      TEST_EQ_APPROX("e *= 2.0", e *= 2.0f, Plane3::plane(Vec3(2.0, -4.0, 6.0), 10.0));
    });
    UNIT_TEST("Scalar division", {
      Plane3 e(a);
      TEST_EQ_APPROX("a / 0.5", a / 0.5f, Plane3::plane(Vec3(2.0, -4.0, 6.0), 10.0));
      TEST_EQ_APPROX("e /= 0.5", e /= 0.5f, Plane3::plane(Vec3(2.0, -4.0, 6.0), 10.0));
    });
    UNIT_TEST("Is vanishing", {
      TEST_EQ("a", is_vanishing(a), false);
      TEST_EQ("b", is_vanishing(b), true);
    });
    UNIT_TEST("Magnitude", {
      TEST_EQ_APPROX("||a||^2", magnitude_squared(a), 14.0f);
      TEST_EQ_APPROX("||a||", magnitude(a), std::sqrt(14.0f));
      TEST_EQ_APPROX("||b||^2", magnitude_squared(b), 0.0f);
      TEST_EQ_APPROX("||b||", magnitude(b), 0.0f);
      TEST_EQ_APPROX("||a||_inf^2", vanishing_magnitude_squared(a), 25.0f);
      TEST_EQ_APPROX("||a||_inf", vanishing_magnitude(a), 5.0f);
      TEST_EQ_APPROX("||b||_inf^2", vanishing_magnitude_squared(b), 4.0f);
      TEST_EQ_APPROX("||b||_inf", vanishing_magnitude(b), 2.0f);
    });
    UNIT_TEST("Normalized", {
      float sqrt14 = std::sqrt(14.0);
      TEST_EQ_APPROX("normalize a", normalized(a), Plane3::plane(Vec3(1.0, -2.0, 3.0) / sqrt14, 5.0 / sqrt14));
      TEST_EQ_APPROX("normalize b", normalized(b), Plane3(0.0, 0.0, 0.0, -1.0));
    });
    UNIT_TEST("Meet 2", {
      TEST_EQ_APPROX("meet(a, b)", meet(a, b), Line3::vanishing_line(Vec3(2.0, -4.0, 6.0)));
      TEST_EQ_APPROX("meet(a, c)", meet(a, c), Line3::from_plucker(Vec3(-4.0, 13.0, 10.0), Vec3(-22.0, -6.0, -1.0)));
      TEST_EQ_APPROX("meet(c, a)", meet(c, a), Line3::from_plucker(Vec3(4.0, -13.0, -10.0), Vec3(22.0, 6.0, 1.0)));
    });
    UNIT_TEST("Meet 3", {
      TEST_EQ_APPROX("meet(a, b, c)", meet(a, b, c), Point3::direction(Vec3(8.0, -26.0, -20.0)));
      TEST_EQ_APPROX("meet(a, c, d)", meet(a, c, d), Point3(-49.0, 156.0, 142.0, 13.0));
    });
    UNIT_TEST("Inner", {
      TEST_EQ_APPROX("inner(a, b)", inner(a, b), 0.0f);
      TEST_EQ_APPROX("inner(a, c)", inner(a, c), -3.0f);
    });
    UNIT_TEST("reverse", {
      TEST_EQ_APPROX("reverse(a)", reverse(a), a);
      TEST_EQ_APPROX("reverse(b)", reverse(b), b);
      TEST_EQ_APPROX("reverse(c)", reverse(c), c);
    });
    UNIT_TEST("inverse", {
      TEST_EQ_APPROX("inverse(a)", inverse(a), a / 14.0f);
      TEST_EQ_APPROX("inverse(c)", inverse(c), c / 21.0f);
    });
    UNIT_TEST("Dual", {
      TEST_EQ_APPROX("dual a", dual(a), Point3(1.0, -2.0, 3.0, 0.0));
      TEST_EQ_APPROX("dual b", dual(b), Point3::ZERO);
    });
  })


  UNIT_TEST_SECTION("Line3", {
    const Line3 a = Line3::line(Vec3(2.0, 1.0, 0.0), Vec3(1.0, 3.0, -2.0));
    const Line3 b = Line3::vanishing_line(Vec3(1.0, 0.0, -2.0));

    UNIT_TEST("Addition", {
      Line3 c(a);
      TEST_EQ_APPROX("a + b", a + b, Line3(2.0, 1.0, 0.0, 1.0, -4.0, -3.0));
      TEST_EQ_APPROX("c += b", c += b, Line3(2.0, 1.0, 0.0, 1.0, -4.0, -3.0));
    });
    UNIT_TEST("Subtraction", {
      Line3 c(b);
      TEST_EQ_APPROX("b - a", b - a, Line3(-2.0, -1.0, 0.0, -3.0, 4.0, 7.0));
      TEST_EQ_APPROX("c -= a", c -= a, Line3(-2.0, -1.0, 0.0, -3.0, 4.0, 7.0));
    });
    UNIT_TEST("Scalar multiplication", {
      Line3 c(a);
      TEST_EQ_APPROX("a * 2.0", a * 2.0f, Line3(4.0, 2.0, 0.0, 4.0, -8.0, -10.0));
      TEST_EQ_APPROX("2.0 * a", 2.0f * a, Line3(4.0, 2.0, 0.0, 4.0, -8.0, -10.0));
      TEST_EQ_APPROX("c *= 2.0", c *= 2.0f, Line3(4.0, 2.0, 0.0, 4.0, -8.0, -10.0));
    });
    UNIT_TEST("Scalar division", {
      Line3 c(a);
      TEST_EQ_APPROX("a / 0.5", a / 0.5f, Line3(4.0, 2.0, 0.0, 4.0, -8.0, -10.0));
      TEST_EQ_APPROX("c /= 0.5", c /= 0.5f, Line3(4.0, 2.0, 0.0, 4.0, -8.0, -10.0));
    });
    UNIT_TEST("Is vanishing", {
      TEST_EQ("a", is_vanishing(a), false);
      TEST_EQ("b", is_vanishing(b), true);
    });
    UNIT_TEST("Magnitude", {
      TEST_EQ_APPROX("||a||^2", magnitude_squared(a), 5.0f);
      TEST_EQ_APPROX("||a||", magnitude(a), std::sqrt(5.0f));
      TEST_EQ_APPROX("||b||^2", magnitude_squared(b), 0.0f);
      TEST_EQ_APPROX("||b||", magnitude(b), 0.0f);
      TEST_EQ_APPROX("||a||_inf^2", vanishing_magnitude_squared(a), 45.0f);
      TEST_EQ_APPROX("||a||_inf", vanishing_magnitude(a), std::sqrt(45.0f));
      TEST_EQ_APPROX("||b||_inf^2", vanishing_magnitude_squared(b), 5.0f);
      TEST_EQ_APPROX("||b||_inf", vanishing_magnitude(b), std::sqrt(5.0f));
    });
    UNIT_TEST("Normalized", {
      float sqrt5 = std::sqrt(5.0);
      float sqrt45 = std::sqrt(45.0f);
      TEST_EQ_APPROX("normalize a", normalized(a), Line3::from_plucker(Vec3(2.0, 1.0, 0.0) / sqrt45, Vec3(2.0, -4.0, -5.0) / sqrt45));
      TEST_EQ_APPROX("normalize b", normalized(b), Line3::from_plucker(Vec3::ZERO, Vec3(-1.0, 0.0, 2.0) / sqrt5));
    });
    UNIT_TEST("Inner", {
      TEST_EQ_APPROX("inner(a, b)", inner(a, b), 0.0f);
      TEST_EQ_APPROX("inner(a, a)", inner(a, a), -5.0f);
    });
    UNIT_TEST("Reverse", {
      TEST_EQ_APPROX("reverse(a)", reverse(a), Line3(-2.0, -1.0, -0.0, -2.0, 4.0, 5.0));
    });
    UNIT_TEST("Inverse", {
      TEST_EQ_APPROX("inverse(a)", inverse(a), Line3(-2.0 / 5.0, -1.0 / 5.0, 0.0, -2.0 / 5.0, 4.0 / 5.0, 5.0 / 5.0));
    });
    UNIT_TEST("meet & join", {
      TEST_EQ_APPROX("meet(a, b)", meet(a, b), -2.0f);
      TEST_EQ_APPROX("meet(b, a)", meet(b, a), -2.0f);
      TEST_EQ_APPROX("join(a, b)", join(a, b), -2.0f);
      TEST_EQ_APPROX("join(b, a)", join(b, a), -2.0f);
    });
  })


  UNIT_TEST_SECTION("Point3", {
    Point3 a = Point3(1.0, 2.0, 3.0, 2.0);
    Point3 b = Point3(-2.0, 1.0, 3.0, 1.0);
    Point3 c = Point3(-3.0, -1.0, 0.0, 0.0);

    UNIT_TEST("Addition", {
      Point3 c(a);
      TEST_EQ_APPROX("a + b", a + b, Point3(-1.0, 3.0, 6.0, 3.0));
      TEST_EQ_APPROX("c += b", c += b, Point3(-1.0, 3.0, 6.0, 3.0));
    });
    UNIT_TEST("Subtraction", {
      Point3 c(b);
      TEST_EQ_APPROX("b - a", b - a, Point3(-3.0, -1.0, 0.0, -1.0));
      TEST_EQ_APPROX("c -= a", c -= a, Point3(-3.0, -1.0, 0.0, -1.0));
    });
    UNIT_TEST("Scalar multiplication", {
      Point3 c(a);
      TEST_EQ_APPROX("a * 2.0", a * 2.0f, Point3(2.0, 4.0, 6.0, 4.0));
      TEST_EQ_APPROX("2.0 * a", 2.0f * a, Point3(2.0, 4.0, 6.0, 4.0));
      TEST_EQ_APPROX("c *= 2.0", c *= 2.0f, Point3(2.0, 4.0, 6.0, 4.0));
    });
    UNIT_TEST("Scalar division", {
      Point3 c(a);
      TEST_EQ_APPROX("a / 0.5", a / 0.5f, Point3(2.0, 4.0, 6.0, 4.0));
      TEST_EQ_APPROX("c /= 0.5", c /= 0.5f, Point3(2.0, 4.0, 6.0, 4.0));
    });
    UNIT_TEST("As vector", {
      TEST_EQ_APPROX("vec a", as_vector(a), Vec3(0.5, 1.0, 1.5));
      TEST_EQ_APPROX("vec c", as_vector(c), Vec3(3.0, 1.0, 0.0));
    });
    UNIT_TEST("Magnitude", {
      TEST_EQ_APPROX("||a||^2", magnitude_squared(a), 4.0f);
      TEST_EQ_APPROX("||a||", magnitude(a), 2.0f);
      TEST_EQ_APPROX("||c||^2", magnitude_squared(c), 0.0f);
      TEST_EQ_APPROX("||c||", magnitude(c), std::sqrt(0.0f));
      TEST_EQ_APPROX("||a||_inf^2", vanishing_magnitude_squared(a), 14.0f);
      TEST_EQ_APPROX("||a||_inf", vanishing_magnitude(a), std::sqrt(14.0f));
      TEST_EQ_APPROX("||c||_inf^2", vanishing_magnitude_squared(c), 10.0f);
      TEST_EQ_APPROX("||c||_inf", vanishing_magnitude(c), std::sqrt(10.0f));
    });
    UNIT_TEST("Is vanishing", {
      TEST_EQ_APPROX("a", is_vanishing(a), false);
      TEST_EQ_APPROX("c", is_vanishing(c), true);
    });
    UNIT_TEST("Normalized", {
      float sqrt10 = std::sqrt(10.0);
      TEST_EQ_APPROX("normalize a", normalized(a), Point3(0.5, 1.0, 1.5, 1.0));
      TEST_EQ_APPROX("normalize b", normalized(b), b);
      TEST_EQ_APPROX("normalize c", normalized(c), Point3(-3.0 / sqrt10, -1.0 / sqrt10, 0.0, 0.0));
    });
    UNIT_TEST("Join 2", {
      TEST_EQ_APPROX("join(a, b)", join(a, b), Line3(5.0, 0.0, -3.0, -3.0, 9.0, -5.0));
      TEST_EQ_APPROX("join(a, c)", join(a, c), Line3(6.0, 2.0, 0.0, -3.0, 9.0, -5.0));
    });
    UNIT_TEST("Join 3", {
      Point3 d = Point3(3.0, 0.0, 5.0, -1.0);
      TEST_EQ_APPROX("join(a, b, c)", join(a, b, c), Plane3(-3.0, 9.0, -5.0, 0.0));
      TEST_EQ_APPROX("join(a, b, d)", join(a, b, d), Plane3(3.0, -43.0, 5.0, 34.0));
    });
    UNIT_TEST("Inner", {
      TEST_EQ_APPROX("inner(a, b)", inner(a, b), -2.0f);
    });
    UNIT_TEST("Reverse", {
      TEST_EQ_APPROX("reverse(a)", reverse(a), Point3(-1.0, -2.0, -3.0, -2.0));
    });
    UNIT_TEST("Inverse", {
      TEST_EQ_APPROX("inverse(a)", inverse(a), Point3(-1.0 / 4.0, -2.0 / 4.0, -3.0 / 4.0, -2.0 / 4.0));
    });
  })


  UNIT_TEST_SECTION("Operations between Euclidian Flat 3D", {
    Plane3 p = Plane3::plane(-1.0, 6.0, 2.0, -4.0);
    Plane3 vp = Plane3::vanishing_plane(-4.0);
    Line3 l = Line3::line(7.0, -4.0, 1.0, 1.0, 6.0, -2.0);
    Line3 vl = Line3::vanishing_line(4.0, -3.0, 1.0);
    Point3 x = Point3::point(2.0, 5.0, -1.0);
    Point3 v = Point3::direction(1.0, -2.0, -4.0);

    UNIT_TEST("Plane-line meet", {
      TEST_EQ_APPROX("meet(p, l)", meet(p, l), Point3(-274.0, -34.0, 23.0, -29.0));
      TEST_EQ_APPROX("meet(l, p)", meet(l, p), Point3(-274.0, -34.0, 23.0, -29.0));

      TEST_EQ_APPROX("meet(p, vl)", meet(p, vl), Point3(-12.0, -9.0, 21.0, 0.0));
      TEST_EQ_APPROX("meet(vl, p)", meet(vl, p), Point3(-12.0, -9.0, 21.0, 0.0));

      TEST_EQ_APPROX("meet(vp, l)", meet(vp, l), Point3(-28.0, 16.0, -4.0, 0.0));
      TEST_EQ_APPROX("meet(l, vp)", meet(l, vp), Point3(-28.0, 16.0, -4.0, 0.0));

      TEST_EQ_APPROX("meet(vp, vl)", meet(vp, vl), Point3(0.0, 0.0, 0.0, 0.0));
      TEST_EQ_APPROX("meet(vl, vp)", meet(vl, vp), Point3(0.0, 0.0, 0.0, 0.0));
    });
    UNIT_TEST("Plane-line inner", {
      TEST_EQ_APPROX("inner(p, l)", inner(p, l), Plane3(-14.0, -15.0, 38.0, 180.0));
      TEST_EQ_APPROX("inner(l, p)", inner(l, p), Plane3(14.0, 15.0, -38.0, -180.0));

      TEST_EQ_APPROX("inner(p, vl)", inner(p, vl), Plane3(0.0, 0.0, 0.0, -20.0));
      TEST_EQ_APPROX("inner(vl, p)", inner(vl, p), Plane3(0.0, 0.0, 0.0, 20.0));

      TEST_EQ_APPROX("inner(vp, l)", inner(vp, l), Plane3(0.0, 0.0, 0.0, 0.0));
      TEST_EQ_APPROX("inner(l, vp)", inner(l, vp), Plane3(0.0, 0.0, 0.0, 0.0));

      TEST_EQ_APPROX("inner(vp, vl)", inner(vp, vl), Plane3(0.0, 0.0, 0.0, 0.0));
      TEST_EQ_APPROX("inner(vl, vp)", inner(vl, vp), Plane3(0.0, 0.0, 0.0, 0.0));
    });
    UNIT_TEST("Line-point join", {
      TEST_EQ_APPROX("join(x, l)", join(x, l), Plane3(-3.0, -6.0, -3.0, 33.0));
      TEST_EQ_APPROX("join(l, x)", join(l, x), Plane3(-3.0, -6.0, -3.0, 33.0));

      TEST_EQ_APPROX("join(x, vl)", join(x, vl), Plane3(-4.0, 3.0, -1.0, -8.0));
      TEST_EQ_APPROX("join(vl, x)", join(vl, x), Plane3(-4.0, 3.0, -1.0, -8.0));

      TEST_EQ_APPROX("join(v, l)", join(v, l), Plane3(-18.0, -29.0, 10.0, 212.0));
      TEST_EQ_APPROX("join(l, v)", join(l, v), Plane3(-18.0, -29.0, 10.0, 212.0));

      TEST_EQ_APPROX("join(v, vl)", join(v, vl), Plane3(0.0, 0.0, 0.0, -6.0));
      TEST_EQ_APPROX("join(vl, v)", join(vl, v), Plane3(0.0, 0.0, 0.0, -6.0));
    });
    UNIT_TEST("Line-point inner", {
      TEST_EQ_APPROX("inner(x, l)", inner(x, l), Plane3(-7.0, 4.0, -1.0, -7.0));
      TEST_EQ_APPROX("inner(l, x)", inner(l, x), Plane3(-7.0, 4.0, -1.0, -7.0));

      TEST_EQ_APPROX("inner(x, vl)", inner(x, vl), Plane3(0.0, 0.0, 0.0, 0.0));
      TEST_EQ_APPROX("inner(vl, x)", inner(vl, x), Plane3(0.0, 0.0, 0.0, 0.0));

      TEST_EQ_APPROX("inner(v, l)", inner(v, l), Plane3(0.0, 0.0, 0.0, -11.0));
      TEST_EQ_APPROX("inner(l, v)", inner(l, v), Plane3(0.0, 0.0, 0.0, -11.0));

      TEST_EQ_APPROX("inner(v, vl)", inner(v, vl), Plane3(0.0, 0.0, 0.0, 0.0));
      TEST_EQ_APPROX("inner(vl, v)", inner(vl, v), Plane3(0.0, 0.0, 0.0, 0.0));
    });
    UNIT_TEST("Plane-point meet", {
      TEST_EQ_APPROX("meet(x, p)", meet(x, p), -30.0f);
      TEST_EQ_APPROX("meet(p, x)", meet(p, x), 30.0f);

      TEST_EQ_APPROX("meet(x, vp)", meet(x, vp), -4.0f);
      TEST_EQ_APPROX("meet(vp, x)", meet(vp, x), 4.0f);

      TEST_EQ_APPROX("meet(v, p)", meet(v, p), -21.0f);
      TEST_EQ_APPROX("meet(p, v)", meet(p, v), 21.0f);

      TEST_EQ_APPROX("meet(v, vp)", meet(v, vp), 0.0f);
      TEST_EQ_APPROX("meet(vp, v)", meet(vp, v), 0.0f);
    });
    UNIT_TEST("Plane-point join", {
      TEST_EQ_APPROX("join(x, p)", join(x, p), 30.0f);
      TEST_EQ_APPROX("join(p, x)", join(p, x), -30.0f);

      TEST_EQ_APPROX("join(x, vp)", join(x, vp), 4.0f);
      TEST_EQ_APPROX("join(vp, x)", join(vp, x), -4.0f);

      TEST_EQ_APPROX("join(v, p)", join(v, p), 21.0f);
      TEST_EQ_APPROX("join(p, v)", join(p, v), -21.0f);

      TEST_EQ_APPROX("join(v, vp)", join(v, vp), 0.0f);
      TEST_EQ_APPROX("join(vp, v)", join(vp, v), 0.0f);
    });
    UNIT_TEST("Plane-point inner", {
      TEST_EQ_APPROX("inner(x, p)", inner(x, p), Line3(-1.0, 6.0, 2.0, 16.0, -3.0, 17.0));
      TEST_EQ_APPROX("inner(p, x)", inner(p, x), Line3(-1.0, 6.0, 2.0, 16.0, -3.0, 17.0));

      TEST_EQ_APPROX("inner(x, vp)", inner(x, vp), Line3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      TEST_EQ_APPROX("inner(vp, x)", inner(vp, x), Line3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

      TEST_EQ_APPROX("inner(v, p)", inner(v, p), Line3(0.0, 0.0, 0.0, -20.0, -2.0, -4.0));
      TEST_EQ_APPROX("inner(p, v)", inner(p, v), Line3(0.0, 0.0, 0.0, -20.0, -2.0, -4.0));

      TEST_EQ_APPROX("inner(v, vp)", inner(v, vp), Line3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      TEST_EQ_APPROX("inner(vp, v)", inner(vp, v), Line3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    });
  })


  UNIT_TEST_SECTION("Color", {
    auto round = [](const Vec3 &v, const int dec = 3){
      const int scale = std::pow(10, dec);
      return Vec3(
        std::roundf(v.x * scale) / scale,
        std::roundf(v.y * scale) / scale,
        std::roundf(v.z * scale) / scale
      );
    };
    
    UNIT_TEST("XYZ to OkLab", {
      TEST_EQ_APPROX("1", round(xyz_to_oklab(XYZD65(0.950f, 1.000f, 1.089f))), OkLab(1.000, 0.000, 0.000));
      TEST_EQ_APPROX("2", round(xyz_to_oklab(XYZD65(1.000f, 0.000f, 0.000f))), OkLab(0.450, 1.236, -0.019));
      TEST_EQ_APPROX("3", round(xyz_to_oklab(XYZD65(0.000f, 1.000f, 0.000f))), OkLab(0.922, -0.671, 0.263));
      TEST_EQ_APPROX("4", round(xyz_to_oklab(XYZD65(0.000f, 0.000f, 1.000f))), OkLab(0.153, -1.415, -0.449));
    });
    UNIT_TEST("OkLab to XYZ", {
      TEST_EQ_APPROX("1", round(oklab_to_xyz(OkLab(1.000, 0.000, 0.000)), 2), XYZD65(0.95f, 1.00f, 1.09f));
      TEST_EQ_APPROX("2", round(oklab_to_xyz(OkLab(0.450, 1.236, -0.019)), 2), XYZD65(1.00f, 0.00f, 0.00f));
      TEST_EQ_APPROX("3", round(oklab_to_xyz(OkLab(0.922, -0.671, 0.263)), 2), XYZD65(0.00f, 1.00f, 0.00f));
      TEST_EQ_APPROX("4", round(oklab_to_xyz(OkLab(0.153, -1.415, -0.449)), 2), XYZD65(0.00f, 0.00f, 1.00f));
    });
  })


  std::cout << "\n" << Testing::get_singleton()->get_final_report() << std::endl;
  bool success = Testing::get_singleton()->has_succeeded();
  Testing::deinit_singleton();
  return (success)? EXIT_SUCCESS : EXIT_FAILURE;
}
