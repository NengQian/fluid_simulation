# 2018 SS Fluid Simulation Lab Repo

## Description

This is the repo for the fluid simulation lab in 2018 SS, made by Chu-I Chao and Neng Qian. It has been tested on Linux(Ubuntu) and Mac OS.

## Building

    mkdir build
    cd build
    cmake ..
    (make clean)
    make

(Optional) Compile in Release mode to get the best performance
> cmake .. -DCMAKE_BUILD_TYPE=Release

## Running the code

We will have 3 executables after the code is built

    ./save_simulation
        output the serialized simulation data
    ./save_fluid_mesh
        take the output from above as input and outputs the serialized mesh data
    ./visualizer
        visualize the serialized simulation data only, OR visualize the serialized simulation and mesh data at the same time. Inputting multiple sets of data is available.

add the flag below for more details
> -h, --help