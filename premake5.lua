project "poly-physx"
language "C++"
cppdialect "C++17"

filter "system:macosx"
   buildoptions {
      "-Wall",
      "-Wextra",
      "-Wpedantic",
      "-Wconversion",
      "-Wno-unused-parameter"
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
   "%{wks.location}/shapes-2D/include",
   "%{wks.location}/rk-integrator/include",
   "%{wks.location}/cpp-kit/include",
   "%{wks.location}/vendor/yaml-cpp/include",
   "%{wks.location}/vendor/glm",
   "%{wks.location}/vendor/spdlog/include"
}
--, "/opt/homebrew/Cellar/libomp/15.0.6/include"}
