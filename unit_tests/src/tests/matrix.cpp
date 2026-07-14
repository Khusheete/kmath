#include "matrix.hpp"
#include "../testing.hpp"

#include "kmath/matrix.hpp"
#include "kmath/constants.hpp"


using namespace kmath;


void test_matrix4() {
  UNIT_TEST("inverse", {
    Mat4 a = Mat4::perspective_rh_no_ndc_hfov(0.3, 50.0, 0.8 * PI, 1.0);
    TEST_EQ_APPROX("a * a^(-1)", a * inverse(a), Mat4::IDENTITY);
  });
}
