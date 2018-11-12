Simulation framework
====================

This README accompanies a series of files which together constitute a framework for the practical course "Fluid Simulation" at RWTH Aachen University. It consists mainly of four parts:

- merely3d, a simple rigid body renderer designed specifically for this course.
- Eigen, a C++ math library.
- Dear ImGui, a well-known and popular GUI library for visualization applications and tools.
- A modified version of CompactNSearch, a small library for efficient neighborhood search.

In addition, some additional files are included to get you started with your simulator. You should look in the files `src/main.cpp`, `src/simulator.hpp` and `src/simulator.cpp` to have an idea of where to start. You're also recommended to use the math types defined in `src/math_types.hpp` for quantities such as position, orientation (quaternion) etc. rather than Eigen types directly, as they avoid some easy sources of bugs when using the Eigen library. See the comment in that file for more information.

Your first step is to make sure that this application compiles. In order to run the code, you will at the very least need `CMake` and a compiler which supports C++11 features.

Building
========

Build instructions for gcc on Linux systems
-------------------------------------

```
# From the root directory of the framework
mkdir build
cd build
cmake ..
make -j4
./simulator
```

You can name your build directory whatever you want, and put it wherever you want, but `build` is a good default.

Build instructions for MSVC on Window
-------------------------------------

You can open `CMakeLists.txt` directly with Visual Studio, which should hopefully allow you to build and run `simulator.exe` directly.

Mac OSX
-------

The framework has not been tested on Mac, but if you have any issues, please let me know.

Other
=====

Release builds for better performance
-------------------------------------

By default, cmake builds with settings similar to a debug build. In order to get more performance out of your application, you may want to set the build type to `Release`. You can do this by setting the CMake variable `CMAKE_BUILD_TYPE=Release`. On Linux, you can do this by invoking `cmake` with the additional argument `-DCMAKE_BUILD_TYPE_RELEASE`, and on Windows you should be able to configure this from the CMake GUI.
