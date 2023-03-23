project "engine"
   language "C++"
   cppdialect "C++20"
   staticruntime "on"
   kind "StaticLib"
   --buildoptions "-Xclang -fopenmp"

   targetdir("../bin/" .. outputdir)
   objdir("../build/" .. outputdir .. "/%{prj.name}")

   files {"src/**.cpp", "include/**.hpp"}

   includedirs "../**/include"--, "/opt/homebrew/Cellar/libomp/15.0.6/include"}
