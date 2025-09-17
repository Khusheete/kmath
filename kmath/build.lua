project "kmath"
  kind "StaticLib"
  language "C++"
  cppdialect "c++20"

  files { "**.cpp", "**.hpp" }
  includedirs { "." }
  
  lib_build_config()
