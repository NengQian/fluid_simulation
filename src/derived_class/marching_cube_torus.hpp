#ifndef MARCHING_CUBE_TORUS
#define MARCHING_CUBE_TORUS

#include "marching_cube.hpp"
#include <math.h>       /* sqrt */
#include <iostream>

using namespace std;

class marching_cube_torus: public marching_cube
{
public:
    marching_cube_torus(float unit_length):marching_cube(unit_length),R(1.0f),r(0.5f){
        cout<<"hi its torus"<<endl;
    }


//protected:
   virtual void compute_vertices_phi() override{
        size_t len = voxel_vertices.size();
        for(size_t i=0; i< len; ++i)
        {
            float x = voxel_vertices[i].position[0];
            float y = voxel_vertices[i].position[1];
            float z = voxel_vertices[i].position[2];

            float temp = sqrt(x*x+y*y)-R;
            voxel_vertices[i].phi = r*r-temp*temp-z*z;
        }
        return;
    }

    virtual void compute_vertex_normal(const Vector3f& vertex, Vector3f& normal) override{
        float x = vertex[0];
        float y = vertex[1];
        float z = vertex[2];

        float temp = sqrt(x*x+y*y);
        float dx = -2*(temp-R)*(x/temp);
        float dy = -2*(temp-R)*(y/temp);
        float dz = -2*z;

        normal = Vector3f(-dx,-dy,-dz).normalized();
    }


    const float R;
    const float r;



};

#endif // MARCHING_CUBE_TORUS

