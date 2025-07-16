project "KmathTest"
  kind "WindowedApp"
  language "C++"
  cppdialect "c++20"

  files { "src/**.hpp", "src/**.cpp" }
  links { "KMath", "raylib" }
  
  app_build_config()
