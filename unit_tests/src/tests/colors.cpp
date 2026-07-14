#include "colors.hpp"
#include "../testing.hpp"

#include "kmath/color/base.hpp"
#include "kmath/color/cie.hpp"
#include "kmath/color/itu.hpp"
#include "kmath/color/ok.hpp"


using namespace kmath;


void test_color_conversion() {
  auto round = [](const Vec3 &v, const int dec = 3){
    const int scale = std::pow(10, dec);
    return Vec3(
      std::roundf(v.x * scale) / scale,
      std::roundf(v.y * scale) / scale,
      std::roundf(v.z * scale) / scale
    );
  };
  
  UNIT_TEST("XYZ to OkLab", {
    TEST_EQ_APPROX("1", round(ok::xyz_to_oklab(cie::XYZ(0.950f, 1.000f, 1.089f))), ok::OkLab(1.000, 0.000, 0.000));
    TEST_EQ_APPROX("2", round(ok::xyz_to_oklab(cie::XYZ(1.000f, 0.000f, 0.000f))), ok::OkLab(0.450, 1.236, -0.019));
    TEST_EQ_APPROX("3", round(ok::xyz_to_oklab(cie::XYZ(0.000f, 1.000f, 0.000f))), ok::OkLab(0.922, -0.671, 0.263));
    TEST_EQ_APPROX("4", round(ok::xyz_to_oklab(cie::XYZ(0.000f, 0.000f, 1.000f))), ok::OkLab(0.153, -1.415, -0.449));
  });
  UNIT_TEST("OkLab to XYZ", {
    TEST_EQ_APPROX("1", round(ok::oklab_to_xyz(ok::OkLab(1.000, 0.000, 0.000)), 2),   cie::XYZ(0.95f, 1.00f, 1.09f));
    TEST_EQ_APPROX("2", round(ok::oklab_to_xyz(ok::OkLab(0.450, 1.236, -0.019)), 2),  cie::XYZ(1.00f, 0.00f, 0.00f));
    TEST_EQ_APPROX("3", round(ok::oklab_to_xyz(ok::OkLab(0.922, -0.671, 0.263)), 2),  cie::XYZ(0.00f, 1.00f, 0.00f));
    TEST_EQ_APPROX("4", round(ok::oklab_to_xyz(ok::OkLab(0.153, -1.415, -0.449)), 2), cie::XYZ(0.00f, 0.00f, 1.00f));
  });
  UNIT_TEST("lRGB to YCbCr", {
    using namespace itu::bt_2020;
    TEST_EQ_APPROX("Black"  , lrgb_to_ycbcr(Lrgb::ZERO)         , Vec3(0     , 0       , 0));
    TEST_EQ_APPROX("Red"    , lrgb_to_ycbcr(Lrgb(1.0, 0.0, 0.0)), Vec3(0.2627, -0.13963, 0.5));
    TEST_EQ_APPROX("Green"  , lrgb_to_ycbcr(Lrgb(0.0, 1.0, 0.0)), Vec3(0.678 , -0.36037, -0.459786));
    TEST_EQ_APPROX("Blue"   , lrgb_to_ycbcr(Lrgb(0.0, 0.0, 1.0)), Vec3(0.0593, 0.5     , -0.0402143));
    TEST_EQ_APPROX("Cyan"   , lrgb_to_ycbcr(Lrgb(0.0, 1.0, 1.0)), Vec3(0.7373, 0.13963 , -0.5));
    TEST_EQ_APPROX("Magenta", lrgb_to_ycbcr(Lrgb(1.0, 0.0, 1.0)), Vec3(0.322 , 0.36037 , 0.459786));
    TEST_EQ_APPROX("Yellow" , lrgb_to_ycbcr(Lrgb(1.0, 1.0, 0.0)), Vec3(0.9407, -0.5    , 0.0402143));
  });
  UNIT_TEST("YCbCr to lRGB", {
    using namespace itu::bt_2020;
    TEST_EQ_APPROX("Black"  , ycbcr_to_lrgb(Vec3(0     , 0       , 0))         , Lrgb::ZERO         );
    TEST_EQ_APPROX("Red"    , ycbcr_to_lrgb(Vec3(0.2627, -0.13963, 0.5))       , Lrgb(1.0, 0.0, 0.0));
    TEST_EQ_APPROX("Green"  , ycbcr_to_lrgb(Vec3(0.678 , -0.36037, -0.459786)) , Lrgb(0.0, 1.0, 0.0));
    TEST_EQ_APPROX("Blue"   , ycbcr_to_lrgb(Vec3(0.0593, 0.5     , -0.0402143)), Lrgb(0.0, 0.0, 1.0));
    TEST_EQ_APPROX("Cyan"   , ycbcr_to_lrgb(Vec3(0.7373, 0.13963 , -0.5))      , Lrgb(0.0, 1.0, 1.0));
    TEST_EQ_APPROX("Magenta", ycbcr_to_lrgb(Vec3(0.322 , 0.36037 , 0.459786))  , Lrgb(1.0, 0.0, 1.0));
    TEST_EQ_APPROX("Yellow" , ycbcr_to_lrgb(Vec3(0.9407, -0.5    , 0.0402143)) , Lrgb(1.0, 1.0, 0.0));
  });
  UNIT_TEST("lRGB to YcCbcCrc", {
    using namespace itu::bt_2020;
    TEST_EQ_APPROX("Black"  , lrgb_to_yccbccrc(Lrgb::ZERO)         , Vec3(0       , 0        , 0));
    TEST_EQ_APPROX("Red"    , lrgb_to_yccbccrc(Lrgb(1.0, 0.0, 0.0)), Vec3(0.503085, -0.259269, 0.500015));
    TEST_EQ_APPROX("Green"  , lrgb_to_yccbccrc(Lrgb(0.0, 1.0, 0.0)), Vec3(0.823632, -0.424465, -0.479358));
    TEST_EQ_APPROX("Blue"   , lrgb_to_yccbccrc(Lrgb(0.0, 0.0, 1.0)), Vec3(0.209015, 0.499991 , -0.121647));
    TEST_EQ_APPROX("Cyan"   , lrgb_to_yccbccrc(Lrgb(0.0, 1.0, 1.0)), Vec3(0.859121, 0.0890512, -0.500012));
    TEST_EQ_APPROX("Magenta", lrgb_to_yccbccrc(Lrgb(1.0, 0.0, 1.0)), Vec3(0.560865, 0.277582 , 0.441875));
    TEST_EQ_APPROX("Yellow" , lrgb_to_yccbccrc(Lrgb(1.0, 1.0, 0.0)), Vec3(0.970172, -0.499985, 0.0300145));
  });
  UNIT_TEST("YcCbcCrc to lRGB", {
    using namespace itu::bt_2020;
    TEST_EQ_APPROX("Black"  , yccbccrc_to_lrgb(Vec3(0       , 0        , 0))        , Lrgb::ZERO);
    TEST_EQ_APPROX("Red"    , yccbccrc_to_lrgb(Vec3(0.503085, -0.259269, 0.500015)) , Lrgb(1.0, 0.0, 0.0));
    TEST_EQ_APPROX("Green"  , yccbccrc_to_lrgb(Vec3(0.823632, -0.424465, -0.479358)), Lrgb(0.0, 1.0, 0.0));
    TEST_EQ_APPROX("Blue"   , yccbccrc_to_lrgb(Vec3(0.209015, 0.499991 , -0.121647)), Lrgb(0.0, 0.0, 1.0));
    TEST_EQ_APPROX("Cyan"   , yccbccrc_to_lrgb(Vec3(0.859121, 0.0890512, -0.500012)), Lrgb(0.0, 1.0, 1.0));
    TEST_EQ_APPROX("Magenta", yccbccrc_to_lrgb(Vec3(0.560865, 0.277582 , 0.441875)) , Lrgb(1.0, 0.0, 1.0));
    TEST_EQ_APPROX("Yellow" , yccbccrc_to_lrgb(Vec3(0.970172, -0.499985, 0.0300145)), Lrgb(1.0, 1.0, 0.0));
  });
}
