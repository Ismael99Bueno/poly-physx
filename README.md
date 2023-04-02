# Poly-Physx

poly-physx is a 2D physics engine that simulates collisions between convex polygons, forces and interactions between entities, and constraints. It is designed to be flexible and easy to use, allowing developers to create complex 2D physics simulations with ease.

## Features

- Collision detection and resolution between convex polygons
- Forces and interactions between entities, including gravity, friction, and spring forces
- Constraints, including distance constraints and hinge constraints
- Integration with the Runge-Kutta method for accurate movement of entities
- Debugging and profiling tools for better development experience

## Dependencies

poly-physx depends on several other projects, all created by the same author:

- vec-2D: a 2D vector implementation with usual operations
- shapes-2D: a polygon geometry library for creating and manipulating convex polygons
- rk-integrator: an implementation of the Runge-Kutta method for integrating the movement of entities
- debug-tools: a set of tools for debugging poly-physx simulations
- profile-tools: a set of tools for profiling poly-physx simulations

## Usage

To use poly-physx, simply include the engine.hpp header in your project. This is typically the only necessary header to include.

poly-physx is built using premake5 as a static library. To use it, you must create a premake5 workspace with a user-implemented entry point that uses the poly-physx library. You can then build the workspace with premake5 to create an executable.

It is advisable to include poly-physx in a user-made repository as a git submodule. This allows you to keep the poly-physx code separate from your own code, making it easier to manage dependencies and track changes.

For more information on how to use poly-physx, please refer to the documentation.

## License

poly-physx is licensed under the MIT License. See LICENSE for more information.
