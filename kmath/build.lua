project "KMath"
  kind "StaticLib"
  language "C++"
  cppdialect "c++20"

  files { "kmath.hpp", "kmath.cpp" }
  
  app_build_config()
