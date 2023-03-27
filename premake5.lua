project "poly-physx"
   language "C++"
   cppdialect "C++20"
   staticruntime "on"
   kind "StaticLib"
   --buildoptions "-Xclang -fopenmp"

   targetdir("bin/" .. outputdir)
   objdir("build/" .. outputdir)

   files {"src/**.cpp", "include/**.hpp"}

   includedirs "../**/include"--, "/opt/homebrew/Cellar/libomp/15.0.6/include"}
