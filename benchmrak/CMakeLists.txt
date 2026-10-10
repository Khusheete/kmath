add_executable(KMathTests
  src/main.cpp
  src/testing.cpp

  src/tests/vector.cpp
  src/tests/euclidian_flat_3d.cpp
  src/tests/matrix.cpp
  src/tests/rotor_3d.cpp
  src/tests/angles.cpp
  src/tests/colors.cpp
)

target_link_libraries(KMathTests kmath raylib kmath_repo_build_options)
