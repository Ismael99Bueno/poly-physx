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
   defines "PPX_MACOS"

filter "system:windows"
   defines "PPX_WINDOWS"
filter {}

pchheader "ppx/pch.hpp"
pchsource "src/pch.cpp"

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
   "%{wks.location}/debug-log-tools/include",
   "%{wks.location}/profile-tools/include",
   "%{wks.location}/vendor/yaml-cpp/include",
   "%{wks.location}/container-view/include",
   "%{wks.location}/vendor/glm",
   "%{wks.location}/vendor/spdlog/include"
}
--, "/opt/homebrew/Cellar/libomp/15.0.6/include"}
