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

- [vec-2D](https://github.com/Ismael99Bueno/vec-2D): A 2D vector implementation with usual operations
- [shapes-2D](https://github.com/Ismael99Bueno/shapes-2D): A polygon geometry library for creating and manipulating convex polygons
- [rk-integrator](https://github.com/Ismael99Bueno/rk-integrator): An implementation of the Runge-Kutta method for integrating the movement of entities
- [debug-tools](https://github.com/Ismael99Bueno/debug-tools): A set of tools for debugging poly-physx simulations
- [profile-tools](https://github.com/Ismael99Bueno/profile-tools): A set of tools for profiling poly-physx simulations
- [ini-parser](https://github.com/Ismael99Bueno/ini-parser): A simple INI file parser that allows for reading and writing the state of the simulation to and from a file
- [vector-view](https://github.com/Ismael99Bueno/vector-view): A header only library for modifying the contents of a std::vector without letting the user to modify its size.

There is a Python script located in the `scripts` folder named `fetch_dependencies.py`. This script will automatically add all the dependencies as git submodules, provided that the user has already created their own repository and included the current project as a git submodule (or at least downloaded it into the repository). To ensure all runs smoothly once the script has been executed, do not rename the folders containing the various dependencies. All external dependencies, those not created by the same author, will be added as submodules within a folder called `vendor`.

## Building and Usage

1. Install both `premake5` and `make` on your system, as they are required for generating build files and compiling the project respectively.
2. Create a new project folder for your entry point, which will contain the `main.cpp` file. Inside this folder, add a premake5 file that links all necessary libraries and sets the executable kind to `ConsoleApp`.
3. In the root directory of the repository, create another premake5 file. This file should define the workspace, include all dependency projects, and specify the various distributions for the project.
4. To compile the entire project, run the `make` command in the terminal. If you wish to build a specific distribution, use the command `make config=the_distribution`.
5. To use poly-physx, simply include the `engine.hpp` header in your project. This is typically the only necessary header to include.

For more information on how to use poly-physx, please refer to the documentation.

## License

poly-physx is licensed under the MIT License. See LICENSE for more information.
