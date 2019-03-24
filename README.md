# 2018 SS Fluid Simulation Lab Repo

## ToDo
Clean and restructure code.

## Description

This is the repo for the fluid simulation lab in 2018 SS at RWTH, made by Chu-I Chao and Neng Qian. It has been tested on Linux(Ubuntu 16.04) and Mac OS.
It includes PBF and WCSPH solvers, and a visualization program based on Merely3D and imgui. It applys Marching cubes to reconstruct fluid surface.

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

## Test programs

Two test programs will be also built. 

    ./test_marching_cube
        test the marching cube on some basic geometrical primitives
    
    ./kernel_test
        unit tests for kernel functions

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

    Using WCSPH
        ./save_simulation -n 1 -m 2 -f 3000 -o <your_simulation_data_file> -c 0 -t 0.001 -z 10
        ./visualizer -s <your_simulation_data_file> -x 0
    Using PBF
        ./save_simulation -n 1 -m 2 -f 300 -o <your_simulation_data_file> -c 1 -t 0.01 -z 1 
        ./visualizer -s <your_simulation_data_file> -x 0
        
#### Dropping a chunk of water inside a pillar (choose smaller n for shorter waiting time, n=5 for example)

    Using WCSPH 
        ./save_simulation -n 10 -m 8 -f 3000 -o <your_simulation_data_file> -c 0 -t 0.001 -z 10 -s 5000
        ./visualizer -s <your_simulation_data_file> -x 0
    Using PBF
        ./save_simulation -n 10 -m 8 -f 300 -o <your_simulation_data_file> -c 1 -t 0.01 -z 1
        ./visualizer -s <your_simulation_data_file> -x 0     
        
### Narrow Dam break

    Using WCSPH
        ./save_simulation -n 10 -m 5 -f 30000 -o <your_simulation_data_file> -c 0 -t 0.001 -z 100
        ./visualizer -s <your_simulation_data_file> -x 0
    Using PBF
        ./save_simulation -n 10 -m 5 -f 3000 -o <your_simulation_data_file> -c 1 -t 0.01 -z 10
        ./visualizer -s <your_simulation_data_file> -x 0

### Dropping a chunk of water on a spherical boundary
    
    Using WCSPH
        ./save_simulation -n 10 -m 9 -f 3000 -o <your_simulation_data_file> -c 0 -t 0.001 -z 10
        ./visualizer -s <your_simulation_data_file> -x 0    
    Using PBF
        ./save_simulation -n 10 -m 9 -f 300 -o <your_simulation_data_file> -c 1 -t 0.01 -z 1
        ./visualizer -s <your_simulation_data_file> -x 0

### Wave simulation

    Using WCSPH
        ./save_simulation -n 10 -m 10 -f 30000 -o <your_simulation_data_file> -c 0 -t 0.001 -z 50
        ./visualizer -s <your_simulation_data_file> -x 0    
    Using PBF
        ./save_simulation -n 10 -m 10 -f 3000 -o <your_simulation_data_file> -c 1 -t 0.01 -z 5
        ./visualizer -s <your_simulation_data_file> -x 0

### Watermill simulation

    Using WCSPH
        ./save_simulation -n 10 -m 12 -f 30000 -o <your_simulation_data_file> -c 0 -t 0.001 -z 50
        ./visualizer -s <your_simulation_data_file> -x 0    
    Using PBF
        ./save_simulation -n 10 -m 12 -f 3000 -o <your_simulation_data_file> -c 1 -t 0.01 -z 5
        ./visualizer -s <your_simulation_data_file> -x 0
        
## Demo Image And Video Link
### Sphere boundary
![alt text](https://github.com/NengQian/fluid_simulation/blob/master/images/shpere_boundary.png )

### Wave simulation
![alt text](https://github.com/NengQian/fluid_simulation/blob/master/images/wave1.png )
![alt text](https://github.com/NengQian/fluid_simulation/blob/master/images/wave2.png )

### Video of Dam Break  
<a href="https://www.youtube.com/watch?feature=player_embedded&v=EG6p_86yMe4
" target="_blank"><img src="http://img.youtube.com/vi/EG6p_86yMe4/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="640" height="480" border="100" /></a>

## Reference
* Nadir Akinci et al. “Versatile rigid-fluid coupling for incompressible SPH”.
* Markus Becker and Matthias Teschner. “Weakly compressible SPH for free surface flows”. 
* Miles Macklin and Matthias M ̈uller. “Position based fluids”. 
* Daniel J Price. “Smoothed particle hydrodynamics and magnetohydrodynamics”. 

* Merely3D (https://github.com/Andlon/merely3d)
* Cereal (https://github.com/USCiLab/cereal)
* CompactNSearch (https://github.com/InteractiveComputerGraphics/CompactNSearch)
* CLI11 (https://github.com/CLIUtils/CLI11)
* Dear imgui (https://github.com/ocornut/imgui)
