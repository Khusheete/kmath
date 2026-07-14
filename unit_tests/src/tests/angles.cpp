#include "angles.hpp"
#include "../testing.hpp"

#include "kmath/vector.hpp"
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


void test_rotor_euler_conversion() {
  UNIT_TEST("rotor / euler", {
    bool ok = true;
    size_t step = 90;
    
    for (size_t i = 1; i < step; i++) {
      const float alpha = PI * i / step - HALF_PI;
      for (size_t j = 1; j < step; j++) {
        const float beta = TAU * j / step - PI;
        for (size_t k = 1; k < step; k++) {
          const float gamma = 2.0f * TAU * k / step - TAU;
          const Vec3 euler(alpha, gamma, beta);
          const Rotor3 rotor = euler_to_rotor(euler);
          const Vec3 back = rotor_to_euler(rotor);

          const Vec3 diff = back - euler;

          if (any(greater(abs(diff), Vec3(1e-4)))) {
            ok = false;
            break;
          }
        }
        if (!ok) break;
      }
      if (!ok) break;
    }

    TEST("YXZ", ok);
  });
}
