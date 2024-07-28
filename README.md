# poly-physx

poly-physx is a 2D physics engine that I started as a hobby project in September 2022. It attempts to implement some of the most prominent features a physics engine has, such as collision detection, constraint solving, island management, etc. When I started, I had no idea how a physics engine was designed (I didn't even know what broad/narrow phase meant). Because of that, I made some pretty bad design choices that required me to re-write many components of the engine throughout development. I also took the "make it so that I can re-use everything I can" approach, which is not inherently bad, but it makes the engine less portable and harder to integrate into other projects.

Regarding performance, I would say it is mediocre compared to other physics engines available. Some of the latest tests I conducted involved 4 rotating tumblers with 1000 bodies each (4000 in total), with roughly 4.5ms per step. The most prominent issue seems to be cache misses throughout the whole application, but the main bottleneck is constraint contact solving, especially due to memory indirection when iterating over a collection of contact pointers (block allocated, yes, but still not ideal). I was about to optimize that part by using tight structs with constraint information until I decided I wanted to start over, so I guess that won't get done.

Because of all this, this project is now discontinued, and I will focus on developing a new physics engine, applying everything I've learned from the beginning.

## Features

- Collision detection and resolution between convex polygons and circles
- Forces and interactions between bodies, including gravity, friction, electric interactions, etc.
- Multiple actuator and constraint joints
- Can be integrated with Runge-Kutta or symplectic Euler methods
- Island management and sleeping
- Multithreaded collision processing and constraint solving through islands
- Easily customizable broad and narrow phase

## Dependencies

poly-physx depends on several other projects:

- [glm](https://github.com/g-truc/glm)
- [geometry](https://github.com/ismawno/geometry)
- [rk-integrator](https://github.com/ismawno/rk-integrator)
- [cpp-kit](https://github.com/ismawno/cpp-kit)
- [yaml-cpp](https://github.com/ismawno/yaml-cpp) (optional)
- [spdlog](https://github.com/gabime/spdlog) (optional)

## Building and Usage

This project is intended to be used as a git submodule within another project (parent repo). A premake file is provided for building and linking poly-physx.

While these build instructions are minimal, this project is primarily for personal use. Although it has been built and tested on multiple machines (MacOS and Windows), it is not necessarily fully cross-platform or easy to build.

## License

poly-physx is licensed under the MIT License. See LICENSE for more information.
