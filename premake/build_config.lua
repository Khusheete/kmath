-- This build configuration script is licensed separately under the MIT license:
-- 
-- Copyright © 2025 Ferdinand Souchet (aka. Khusheete)
-- 
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the “Software”), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is furnished
-- to do so, subject to the following conditions:
-- 
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
-- 
-- THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
-- INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
-- PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
-- HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
-- CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
-- OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


-- Include premake plugins
require "premake/ecc/ecc"

-- Get host system information. This is usefull for cross compilation.
local syshost = "unknown"
local ver = os.getversion()
local hosttype = ver.description

if hosttype:sub(1,5) == "Linux" then
  syshost = "Linux"
elseif hosttype:sub(1,8) == "Mac OS X" then
  syshost = "MacOSX"
elseif hosttype:sub(1,7) == "Windows" then
  syshost = "Nt"
else
  syshost = hosttype
end


-- Default build configurations
function _build_config()
  -- Include
  includedirs { "%{wks.location}" }
  libdirs { "%{wks.location}/lib" }

  -- Build output
  objdir "%{wks.location}/obj/%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}"

  -- Platform specific configuration
  filter "platforms:linux"
    system "linux"
    toolset "gcc"
    defines { "PLATFORM_LINUX" }
  
  filter "platforms:windows"
    system "windows"
    if syshost == "Linux" then
      gccprefix "x86_64-w64-mingw32-"
      toolset "gcc"
    elseif syshost == "Nt" then
      toolset "msc"
    end
    buildoptions { "-static", "-static-libgcc", "-static-libstdc++" }
    defines { "PLATFORM_WINDOWS" }
  
  -- Debug and release configuration
  filter "configurations:debug"
    symbols "Full"
  
  filter "configurations:release-debug or release"
    defines { "NDEBUG" }
    optimize "On"
  
  filter "configurations:release-debug"
    symbols "Full"
  
  filter ""
end


function app_build_config()
  targetdir "%{wks.location}/bin/%{cfg.platform}/%{cfg.buildcfg}"
  _build_config()
end


function lib_build_config()
  targetdir "%{wks.location}/lib/%{cfg.platform}/%{cfg.buildcfg}"
  _build_config()
end

