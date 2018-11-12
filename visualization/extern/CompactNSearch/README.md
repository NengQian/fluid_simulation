# CompactNSearch

**CompactNSearch** is a C++ library for **parallel** computation of neighboring points in a **fixed radius** in a **three-dimensional point cloud**. The implemented algorithm is a variant of the *Compact Hashing* approach proposed by Ihmsen et al. [IABT11]. The neighborhood information can be efficiently updated when the points spatially move. Moreover, the library offers the possibility to reorder the points (and other array-stored per point information) according to a space-filling Z curve to improve the cache efficiency in later queries or accesses.

The library was used to generate all results of the SPH-based fluid simulations presented by Bender and Koschier in [BK15, BK16].

**Author**: Dan Koschier, **License**: MIT

## Libraries using CompactNSearch
* [PBD] - A C++ library for physically-based simulation of rigid bodies, deformables, cloth and fluids using Position-Based Dynamics
* [SPlisHSPlasH] - A C++ library for the physically-based simulation of fluids using Smoothed Particle Hydrodynamics (cf. video) (Coming soon)

[![Video](https://img.youtube.com/vi/POnmzzhc5E0/0.jpg)](https://www.youtube.com/watch?v=POnmzzhc5E0)

## Build Instructions

This project is based on [CMake](https://cmake.org/). Simply generate project, Makefiles, etc. using [CMake](https://cmake.org/) and compile the project with the compiler of your choice. The code was tested with the following configurations:
- Windows 10 64-bit, CMake 3.7, Visual Studio 2015
- Debian 8 64-bit, CMake 3.7, GCC 4.9.2.

## Usage
A data structure to perform a neighborhood search can be created by calling the constructor given a fixed search radius ```r```.
```c++
CompactNSearch::NeighborhoodSearch nsearch(r);
```
An arbitrary number of point clouds can then be added to the data structure using the method ```add_point_set```. The library expects the point positions to be contiguously stored in an array-like structure. The method will return a unique id associated with the initialized point set.
```c++
std::vector<std::array<Real, 3>> positions;
// ... Fill array with 3 * n real numbers representing three-dimensional point positions.
unsigned int point_set_id = nsearch.add_point_set(positions.front().data(), positions.size());
nsearch.find_neighbors();
```
In order to generate the neighborhood information simply execute the following command
```c++
nsearch.find_neighbors();
```
Finally, the neighborhood information can be accessed as follows
```c++
PointSet const& ps = nsearch.point_set(point_set_id);
for (int i = 0; i < ps.n_points(); ++i)
{
	for (int j = 0; j < ps.n_neighbors(i); ++j)
	{
    	// Return PointID of the jth neighbor of the ith particle in the 0th point set.
	    PointID const& pid = ps.neighbor(0, i, j);
	    // ...
	    // Do whatever you want with the point id. The id contains two indices.
	    // The first field pid.point_set_id represents the unique point set id returnd by add_point_set.
	    // The second field pid.point_id stands for the index of the neighboring particle within
	    // the containing point set.
	    // ...
	}
}
```

Besides the basic functionality the library offers to compute a rule for reordering the points according to a space-filling Z curve. The reordering will improve the performance of future neighborhood queries and accesses. The rule can be computed via
```c++
nsearch.z_sort();
```
Please note that the actual reordering must be invoked by the user by
```c++
ps.sort_field(positions.data());
```
Assuming that there is additional information stored per-point (e.g. velocity, color, mass etc.) the information **must** also be reorded using the same method to maintain consistency. Subsequently, the ```find_neighbors``` function has to be invoked again to update the neighborhood information.

Another self-explaining (benchmark) [demo](demo/main.cpp) is contained in the project.

## Activation Table

When maintaining multiple it is sometimes desired that only certain point sets can find points from other point sets. Therefore an activation table is implemented where the user can specify whether a point set i searches points in another point set j. When nothing else is specified all point sets will search points in all other point sets. The activation table can be modified with e.g.
```c++
nsearch.set_active(i, j, false)
```

## References

* [IABT11] M. Ihmsen, N. Akinci, M. Becker and M. Teschner, 2011. "A Parallel SPH Implementation on Multi-Core CPUs", Computer Graphics Forum 30, 1, 99-112.
* [BK15] J. Bender and D. Koschier 2015. "Divergence-Free Smoothed Particle Hydrodynamics", ACM SIGGRAPH / Eurographics Symposium on Computer Animation, 1-9
* [BK17] J. Bender and D. Koschier, 2017. "Divergence-Free SPH for Incompressible and Viscous Fluids", IEEE Transactions on Visualization and Computer Graphics.

[PBD]: <https://github.com/InteractiveComputerGraphics/PositionBasedDynamics>
[SPlisHSPlasH]: <https://github.com/InteractiveComputerGraphics/SPlisHSPlasH>
