project "poly-physx"
language "C++"
cppdialect "c++20"

filter "system:macosx"
   buildoptions {
      "-Wall",
      "-Wextra",
      "-Wpedantic",
      "-Wconversion",
      "-Wno-unused-parameter",
      "-Wno-sign-conversion"
   }
filter {}

pchheader "ppx/internal/pch.hpp"
pchsource "src/internal/pch.cpp"

staticruntime "off"
kind "StaticLib"
--buildoptions "-Xclang -fopenmp"

targetdir("bin/" .. outputdir)
objdir("build/" .. outputdir)

files {
   "src/**.cpp",
   "include/**.hpp"
}

includedirs {
   "include",
   "%{wks.location}/geometry/include",
   "%{wks.location}/rk-integrator/include",
   "%{wks.location}/cpp-kit/include",
   "%{wks.location}/vendor/yaml-cpp/include",
   "%{wks.location}/vendor/glm",
   "%{wks.location}/vendor/spdlog/include"
}
--, "/opt/homebrew/Cellar/libomp/15.0.6/include"}
