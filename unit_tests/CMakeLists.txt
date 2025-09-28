add_executable(KMathTests
  src/main.cpp
  src/testing.cpp
)

target_link_libraries(KMathTests kmath raylib kmath_repo_build_options)
