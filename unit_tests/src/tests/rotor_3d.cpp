#include "rotor_3d.hpp"
#include "../testing.hpp"

#include "kmath/rotor_3d.hpp"


using namespace kmath;


void test_rotor3() {
  UNIT_TEST("sqrt", {
    const Rotor3 a = Rotor3::from_axis_angle(normalized(Vec3(1.0, 2.0, -0.2)), PI * 0.1f);
    const Rotor3 sqrt_a = sqrt(a);
    TEST_EQ_APPROX("sqrt(a) * sqrt(a)", sqrt_a * sqrt_a, a);
  });
}
