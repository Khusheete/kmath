add_executable(KMathExamples
  src/main.cpp
  # Examples sources
  src/rotor_motor.cpp
  src/motor_transforms.cpp
  src/ease_function.cpp
  src/pga_visualization.cpp
  src/oklab_interpolation.cpp
  src/camera.cpp
  # Utils sources
  src/utils/freecam.cpp
  src/utils/math.cpp
  src/utils/pga.cpp
)

target_link_libraries(KMathExamples kmath raylib kmath_repo_build_options)
