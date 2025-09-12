workspace "build-template"
  configurations { "debug", "release-debug", "release" }
  platforms { "linux", "windows" }


-- Build configuration script
include "premake/build_config.lua"


-- Include build files
-- include "thirdparty/build.lua"
include "examples/build.lua"
include "unit_tests/build.lua"
