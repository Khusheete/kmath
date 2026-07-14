#include "testing.hpp"

#include "unit_tests/src/csi.hpp"
#include "unit_tests/src/tests/angles.hpp"
#include "unit_tests/src/tests/euclidian_flat_3d.hpp"
#include "unit_tests/src/tests/matrix.hpp"
#include "unit_tests/src/tests/rotor_3d.hpp"
#include "unit_tests/src/tests/vector.hpp"
#include "unit_tests/src/tests/colors.hpp"

#include <array>
#include <set>
#include <cstdlib>
#include <cstring>
#include <ostream>
#include <string_view>


using TestFunctionPtr = void(*)();

struct TestSection {
  const std::string_view name;
  const TestFunctionPtr function;
};


constexpr const std::array<TestSection, 11> TEST_SECTIONS{
  // TODO: add more tests for colors
  // TODO: test operations on rotors
  // TODO: test operations on motors
  // TODO: test operations on matrices

  TestSection{ .name = "vec2", .function = &test_vector2, },
  TestSection{ .name = "vec3", .function = &test_vector3, },
  TestSection{ .name = "vec4", .function = &test_vector4, },

  TestSection{ .name = "plane3", .function = &test_plane3, },
  TestSection{ .name = "line3", .function = &test_line3, },
  TestSection{ .name = "point3", .function = &test_point3, },
  TestSection{ .name = "cross_flat3", .function = &test_cross_flat3_operations, },

  TestSection{ .name = "matrix4", .function = &test_matrix4, },

  TestSection{ .name = "rotor3", .function = &test_rotor3, },

  TestSection{ .name = "color_conversion", .function = &test_color_conversion, },

  TestSection{ .name = "rotor3_euler_conversion", .function = &test_rotor_euler_conversion, },
};


void help() {
  std::cout << "Unit tests for the kmath library.\n";
  std::cout << "\n";

  std::cout << "Usage:\n";
  std::cout << "\tKMathTests all\t\t\t- execute all tests\n";
  std::cout << "\tKMathTests <test_sections>\t\t- execute every test in the list\n";
  std::cout << "\n";

  std::cout << "List of tests sections:\n";
  for (const TestSection &section : TEST_SECTIONS) {
    std::cout << "\t" << section.name << "\n";
  }

  std::flush(std::cout);
}


int main(const int argc, const char *const *const argv) {
  if (argc <= 1) {
    help();
    return EXIT_SUCCESS;
  }

  Testing::init_singleton();
  if (std::strcmp(argv[1], "all") == 0) { // Execute all tests
    for (const TestSection &section : TEST_SECTIONS) {
      UNIT_TEST_SECTION(section.name, { section.function(); });
    }
  } else { // Execute only the listed tests
    std::set<std::string_view> selected_sections;
    for (int i = 1; i < argc; i++) {
      selected_sections.emplace(argv[i]);
    }

    for (const TestSection &section : TEST_SECTIONS) {
      if (selected_sections.contains(section.name)) {
        UNIT_TEST_SECTION(section.name, { section.function(); });
        selected_sections.erase(section.name);
      }
    }

    if (!selected_sections.empty()) {
      std::cerr << CSI_RED << "The following test sections were given but do not exist:\n";
      for (const std::string_view name : selected_sections) {
        std::cerr << "\t" << name << "\n";
      }
      std::cerr << CSI_CLEAR;
      std::flush(std::cerr);
    }
  }

  std::cout << "\n" << Testing::get_singleton()->get_final_report() << std::endl;
  const bool success = Testing::get_singleton()->has_succeeded();
  Testing::deinit_singleton();
  return (success)? EXIT_SUCCESS : EXIT_FAILURE;
}
