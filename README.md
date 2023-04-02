# poly-physx

poly-physx is a 2D physics engine that simulates collisions between convex polygons, forces and interactions between entities, and constraints. It is designed to be flexible and easy to use, allowing developers to create complex 2D physics simulations with ease.

## Features

- Collision detection and resolution between convex polygons
- Forces and interactions between entities, including gravity, friction, and spring forces
- Constraints, including distance constraints and hinge constraints
- Integration with the Runge-Kutta method for accurate movement of entities
- Debugging and profiling tools for better development experience
- Ability to write and read the state of the simulation to and from a file using an ini-parser

## Dependencies

poly-physx depends on several other projects, all created by the same author:

- [vec-2D](https://github.com/Ismael99Bueno/vec-2D): a 2D vector implementation with usual operations
- [shapes-2D](https://github.com/Ismael99Bueno/shapes-2D): a polygon geometry library for creating and manipulating convex polygons
- [rk-integrator](https://github.com/Ismael99Bueno/rk-integrator): an implementation of the Runge-Kutta method for integrating the movement of entities
- [debug-tools](https://github.com/Ismael99Bueno/debug-tools): a set of tools for debugging poly-physx simulations
- [profile-tools](https://github.com/Ismael99Bueno/profile-tools): a set of tools for profiling poly-physx simulations
- [ini-parser](https://github.com/Ismael99Bueno/ini-parser): a simple INI file parser that allows for reading and writing the state of the simulation to and from a file
- [vector-view](https://github.com/Ismael99Bueno/vector-view): A header only library for modifying the contents of a std::vector without letting the user to modify its size.

## Usage

To use poly-physx, simply include the engine.hpp header in your project. This is typically the only necessary header to include.

poly-physx is built using premake5 as a static library. To use it, you must create a premake5 workspace with a user-implemented entry point that uses the poly-physx library. You can then build the workspace with premake5 to create an executable.

It is advisable to include poly-physx in a user-made repository as a git submodule. This allows you to keep the poly-physx code separate from your own code, making it easier to manage dependencies and track changes.

For more information on how to use poly-physx, please refer to the documentation.

## License

poly-physx is licensed under the MIT License. See LICENSE for more information.
