#ifndef MARCHING_CUBE_SPHERE
#define MARCHING_CUBE_SPHERE

#include "marching_cube.hpp"
#include <math.h>       /* sqrt */
#include <iostream>

using namespace std;

class marching_cube_sphere: public marching_cube
{
public:
    marching_cube_sphere(float unit_length) : marching_cube(unit_length)
	{
        ;
    }


//protected:
    // should define according to the data
    virtual void compute_vertex_normal(const Vector3f& vertex, Vector3f& normal) override
    {// this is for the sphere, which center is the origin points
        Vector3f ori = {0.0f,0.0f,0.0f};
        normal = (vertex - ori).normalized();

        // debug
    }

    // this function now only used to compute the unit sphere.
    virtual void compute_vertices_phi() override
    {
        size_t len = voxel_vertices.size();
        for(size_t i=0; i<len; ++i)
        {
            float dist = voxel_vertices[i].position.norm();
            voxel_vertices[i].phi = 1 - dist;
        }
        return;
    }
};

#endif // MARCHING_CUBE_SPHERE

