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

We will have 3 executables after the code is built.

    ./save_simulation
        output the serialized simulation data
    ./save_fluid_mesh
        take the output from above as input and outputs the serialized mesh data
    ./visualizer
        visualize the serialized simulation data only, OR visualize the serialized simulation and mesh data at the same time. Inputting multiple sets of data is available.

add the flag below for more details
> -h, --help

## Selective test scenarios

To reproduce the results we've mentioned in the final report, we suggest some terminal commands here.
Here we assume that the program is built successfully and you are in the ./build directory.

#### 2-cube collision

    Using WCSPH
        ./save_simulation -n 5 -m 4 -f 5000 -o <your_simulation_data_file> -c 0 -t 0.001 -z 10
        ./visualizer -s <your_simulation_data_file> -x 0
    Using PBF
        ./save_simulation -n 5 -m 4 -f 500 -o <your_simulation_data_file> -c 1 -t 0.01 -z 1
        ./visualizer -s <your_simulation_data_file> -x 0    

#### Dropping single particle on the boundary
#### Dropping a chunk of water inside a pillar