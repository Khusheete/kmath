cmake_minimum_required(VERSION 3.15)


project(kmath VERSION 1.0)


# == Compilation options ==

option(DEBUG_BUILD "Build project with debug symbols" OFF)


# == Project options ==
set(CMAKE_EXPORT_COMPILE_COMMANDS True)


# == Build options ==
add_library(kmath_repo_build_options INTERFACE)
target_compile_features(kmath_repo_build_options INTERFACE
  cxx_std_20
)
target_link_directories(kmath_repo_build_options INTERFACE
  lib
)
target_include_directories(kmath_repo_build_options INTERFACE "${CMAKE_SOURCE_DIR}")

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,Clang,GNU>")
if(gcc_like_cxx) # Options for gcc ang clang
  target_compile_options(kmath_repo_build_options INTERFACE
    -Wall -Wextra -O3
  )

  if(DEBUG_BUILD)
    target_compile_options(kmath_repo_build_options INTERFACE
      -ggdb
    )
  else()
    target_compile_definitions(kmath_repo_build_options INTERFACE "NDEBUG")
  endif()
endif()


# == Subdirs ==
add_subdirectory(kmath/)
add_subdirectory(unit_tests/)
add_subdirectory(examples/)
