#include "angles.hpp"
#include "../testing.hpp"

#include <array>

#include "kmath/base.hpp"
#include "kmath/rotor_3d.hpp"
#include "kmath/vector.hpp"
#include "kmath/matrix.hpp"
#include "kmath/angles.hpp"


using namespace kmath;


void test_spherical_angles() {
  UNIT_TEST("cartesian to spherical", {
    TEST_EQ_APPROX("(1, 0, 0)" , cartesian_to_spherical(Vec3(1.0, 0.0, 0.0)) , Vec3(1.0, 0.5 * PI, 0.0));
    TEST_EQ_APPROX("(0, 1, 0)" , cartesian_to_spherical(Vec3(0.0, 2.0, 0.0)) , Vec3(2.0, 0.0, 0.0));
    TEST_EQ_APPROX("(0, 0, 1)" , cartesian_to_spherical(Vec3(0.0, 0.0, 1.0)) , Vec3(1.0, 0.5 * PI, 0.5 * PI));
    TEST_EQ_APPROX("(-1, 0, 0)", cartesian_to_spherical(Vec3(-2.0, 0.0, 0.0)), Vec3(2.0, 0.5 * PI, PI));
    TEST_EQ_APPROX("(0, -1, 0)", cartesian_to_spherical(Vec3(0.0, -1.0, 0.0)), Vec3(1.0, PI, 0.0));
    TEST_EQ_APPROX("(0, 0, -1)", cartesian_to_spherical(Vec3(0.0, 0.0, -3.0)), Vec3(3.0, 0.5 * PI, -0.5 * PI));
  });
  UNIT_TEST("spherical to cartesian", {
    TEST_EQ_APPROX("(0, PI, PI / 2)", spherical_to_cartesian(Vec3(0.0, PI, 0.5 * PI)), Vec3::ZERO);
    TEST_EQ_APPROX("(1, PI, PI / 2)", spherical_to_cartesian(Vec3(1.0, PI, 0.5 * PI)), Vec3(0.0, -1.0, 0.0));
    TEST_EQ_APPROX("(1, PI / 2, PI / 2)", spherical_to_cartesian(Vec3(1.0, 0.5 * PI, 0.5 * PI)), Vec3(0.0, 0.0, 1.0));
    TEST_EQ_APPROX("(2, PI / 2, 0)", spherical_to_cartesian(Vec3(2.0, 0.5 * PI, 0.0)), Vec3(2.0, 0.0, 0.0));
  });
}


namespace {
  bool _test_rotor_euler_conversion_interior_space(const Vec3i &permutation, const bool tait_bryan, const EulerBasis basis, const size_t steps = 50) {
    const float divisor = 1.0f / float(steps + 1);

    for (size_t i = 1; i <= steps; i++) {
      const float alpha = PI * i * divisor - float((tait_bryan)? HALF_PI : 0);
      for (size_t j = 1; j <= steps; j++) {
        const float beta = TAU * j * divisor - PI;
        for (size_t k = 1; k <= steps; k++) {
          const float gamma = 2.0f * TAU * k * divisor - TAU;

          const Vec3 angles{alpha, beta, gamma};
          const Vec3 euler{
            angles[permutation.x],
            angles[permutation.y],
            angles[permutation.z],
          };

          const Rotor3 rotor = euler_to_rotor(euler, basis);
          const Vec3 inverse_euler = rotor_to_euler(rotor, basis);
          const Vec3 error = inverse_euler - euler;

          if (any(greater(abs(error), Vec3(1e-4)))) {
            return false;
          }
        }
      }
    }

    return true;
  }


  bool _test_rotor_euler_conversion_guimbal_lock(const Vec3i &permutation, const bool tait_bryan, const EulerBasis basis, const size_t steps = 50) {
    const float divisor = 1.0f / float(steps);

    const int unlocked_coordinate = (permutation.x == 2)? 0 : (permutation.y == 2)? 1 : 2;
    const int guimbal_coordinate = (permutation.x == 0)? 0 : (permutation.y == 0)? 1 : 2;
    const int zeroed_coordinate = 3 - unlocked_coordinate - guimbal_coordinate;

    for (size_t side = 0; side <= 1; side++) {
      for (size_t i = 0; i <= steps; i++) {
        const Vec3 angles{
          side * float(PI) - float((tait_bryan)? HALF_PI : 0),
          0.0f,
          2.0f * float(TAU) * i * divisor - float(TAU),
        };
        const Vec3 euler{
          angles[permutation.x],
          angles[permutation.y],
          angles[permutation.z],
        };

        const Rotor3 rotor = euler_to_rotor(euler, basis);
        const Vec3 inverse_euler = rotor_to_euler(rotor, basis);
        const Vec3 error{
          angles.x - inverse_euler[guimbal_coordinate],
          inverse_euler[zeroed_coordinate],
          angle_difference(angles.z, inverse_euler[unlocked_coordinate]),
        };

        if (any(greater(abs(error), Vec3(1e-4)))) {
          return false;
        }
      }
    }

    return true;
  }


  bool _test_rotor_euler_conversion_non_continuous_boundary(const Vec3i &permutation, const bool tait_bryan, const EulerBasis basis, const size_t steps = 50) {
    const float divisor = 1.0f / float(steps + 1);

    const std::array<Mat3, 4> CASES{
      Mat3(
        Vec3(1, 0, 0),
        Vec3(0, 1, 0),
        Vec3(0, 0, -TAU)
      ),
      Mat3(
        Vec3(1, 0, 0),
        Vec3(0, 1, 0),
        Vec3(0, 0, +TAU)
      ),
      Mat3(
        Vec3(1, 0, 0),
        Vec3(0, 0, 1),
        Vec3(0, -PI, 0)
      ),
      Mat3(
        Vec3(1, 0, 0),
        Vec3(0, 0, 1),
        Vec3(0, +PI, 0)
      ),
    };

    for (size_t i = 1; i <= steps; i++) {
      const float alpha = PI * i * divisor - float((tait_bryan)? HALF_PI : 0);
      for (size_t j = 1; j <= steps; j++) {
        const float beta = TAU * j * divisor - PI;
        for (const Mat3 &c : CASES) {
          const Vec3 angles = c * Vec3(alpha, beta, 1);
          const Vec3 euler{
            angles[permutation.x],
            angles[permutation.y],
            angles[permutation.z],
          };

          const Rotor3 rotor = euler_to_rotor(euler, basis);
          const Vec3 inverse_euler = rotor_to_euler(rotor, basis);
          const Vec3 error = angle_difference(inverse_euler, euler);

          if (any(greater(abs(error), Vec3(1e-4)))) {
            return false;
          }
        }
      }
    }

    return true;
  }
}


void test_rotor_euler_conversion() {
  // == XYZ ==
  UNIT_TEST("XYZ conversion", {
    const Vec3i permutation(2, 0, 1);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, true, EulerBasis::XYZ));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, true, EulerBasis::XYZ));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, true, EulerBasis::XYZ));
  });
  // == YZX ==
  UNIT_TEST("YZX conversion", {
    const Vec3i permutation(1, 2, 0);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, true, EulerBasis::YZX));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, true, EulerBasis::YZX));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, true, EulerBasis::YZX));
  });
  // == ZXY ==
  UNIT_TEST("ZXY conversion", {
    const Vec3i permutation(0, 1, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, true, EulerBasis::ZXY));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, true, EulerBasis::ZXY));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, true, EulerBasis::ZXY));
  });
  // == XZY ==
  UNIT_TEST("XZY conversion", {
    const Vec3i permutation(2, 1, 0);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, true, EulerBasis::XZY));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, true, EulerBasis::XZY));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, true, EulerBasis::XZY));
  });
  // == ZYX ==
  UNIT_TEST("ZYX conversion", {
    const Vec3i permutation(1, 0, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, true, EulerBasis::ZYX));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, true, EulerBasis::ZYX));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, true, EulerBasis::ZYX));
  });
  // == YXZ ==
  UNIT_TEST("YXZ conversion", {
    const Vec3i permutation(0, 2, 1);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, true, EulerBasis::YXZ));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, true, EulerBasis::YXZ));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, true, EulerBasis::YXZ));
  });

  // == ZXZ ==
  UNIT_TEST("ZXZ conversion", {
    const Vec3i permutation(1, 0, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, false, EulerBasis::ZXZ));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, false, EulerBasis::ZXZ));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, false, EulerBasis::ZXZ));
  });
  // == XYX ==
  UNIT_TEST("XYX conversion", {
    const Vec3i permutation(1, 0, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, false, EulerBasis::XYX));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, false, EulerBasis::XYX));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, false, EulerBasis::XYX));
  });
  // == YZY ==
  UNIT_TEST("YZY conversion", {
    const Vec3i permutation(1, 0, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, false, EulerBasis::YZY));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, false, EulerBasis::YZY));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, false, EulerBasis::YZY));
  });
  // == ZYZ ==
  UNIT_TEST("ZYZ conversion", {
    const Vec3i permutation(1, 0, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, false, EulerBasis::ZYZ));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, false, EulerBasis::ZYZ));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, false, EulerBasis::ZYZ));
  });
  // == XZX ==
  UNIT_TEST("XZX conversion", {
    const Vec3i permutation(1, 0, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, false, EulerBasis::XZX));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, false, EulerBasis::XZX));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, false, EulerBasis::XZX));
  });
  // == YXY ==
  UNIT_TEST("YXY conversion", {
    const Vec3i permutation(1, 0, 2);
    TEST("interior space conversion", _test_rotor_euler_conversion_interior_space(permutation, false, EulerBasis::YXY));
    TEST("guimbal lock conversion", _test_rotor_euler_conversion_guimbal_lock(permutation, false, EulerBasis::YXY));
    TEST("on the non continuous boundary", _test_rotor_euler_conversion_non_continuous_boundary(permutation, false, EulerBasis::YXY));
  });
}
