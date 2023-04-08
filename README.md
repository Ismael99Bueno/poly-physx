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

- [vec-2D](https://github.com/ismawno/vec-2D): A 2D vector implementation with usual operations
- [shapes-2D](https://github.com/ismawno/shapes-2D): A polygon geometry library for creating and manipulating convex polygons
- [rk-integrator](https://github.com/ismawno/rk-integrator): An implementation of the Runge-Kutta method for integrating the movement of entities
- [debug-tools](https://github.com/ismawno/debug-tools): A set of tools for debugging poly-physx simulations
- [profile-tools](https://github.com/ismawno/profile-tools): A set of tools for profiling poly-physx simulations
- [ini-parser](https://github.com/ismawno/ini-parser): A simple INI file parser that allows for reading and writing the state of the simulation to and from a file
- [vector-view](https://github.com/ismawno/vector-view): A header only library for modifying the contents of a std::vector without letting the user to modify its size.

There is a Python script located in the `scripts` folder named `fetch_dependencies.py`. This script will automatically add all the dependencies as git submodules, provided that the user has already created their own repository and included the current project as a git submodule (or at least downloaded it into the repository). To ensure all runs smoothly once the script has been executed, do not rename the folders containing the various dependencies. All external dependencies, those not created by the same author, will be added as submodules within a folder called `vendor`.

## Building and Usage

1. Ensure you have `premake5` and `make` installed on your system. `premake5` is used to generate the build files, and `make` is used to compile the project.
2. Create your own repository and include the current project as a git submodule (or at least download it into the repository).
3. Run the `fetch_dependencies.py` script located in the `scripts` folder to automatically add all the dependencies as git submodules.
4. Create an entry point project with a `premake5` file, where the `main.cpp` will be located. Link all libraries and specify the kind of the executable as `ConsoleApp`. Don't forget to specify the different configurations for the project.
5. Create a `premake5` file at the root of the repository describing the `premake` workspace and including all dependency projects.
6. Build the entire project by running the `make` command in your terminal. You can specify the configuration by using `make config=the_configuration`.
7. To use poly-physx, simply include the `engine.hpp` header in your project. This is typically the only necessary header to include.

For more information on how to use poly-physx, please refer to the documentation.

## License

poly-physx is licensed under the MIT License. See LICENSE for more information.
