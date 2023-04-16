project "poly-physx"
   language "C++"
   cppdialect "C++17"
   filter "system:macosx"
      buildoptions {"-Wall", "-Wextra", "-Wpedantic", "-Wconversion", "-Wno-unused-parameter"}
   filter{}
   
   staticruntime "off"
   kind "StaticLib"
   --buildoptions "-Xclang -fopenmp"

   targetdir("bin/" .. outputdir)
   objdir("build/" .. outputdir)

   files {"src/**.cpp", "include/**.hpp"}

   includedirs {"../**/include", "../vendor/glm"}--, "/opt/homebrew/Cellar/libomp/15.0.6/include"}
