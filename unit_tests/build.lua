project "KmathTests"
  kind "WindowedApp"
  language "C++"
  cppdialect "c++20"

  files { "src/**.hpp", "src/**.cpp" }
  links { "kmath", "raylib" }
  
  app_build_config()
