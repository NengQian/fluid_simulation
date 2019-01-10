
#pragma once

#include <merely3d/frame.hpp>
#include <iostream>
#include <fstream>
#include <string.h>
#include "imgui/imgui.h"
#include "math_types.hpp"




#include <array>



// each three indices represent a triangle face
// here we have 4 faces
// note the vertices order of one triangle face should be counter-clock wise

const std::array<unsigned int, 12> pyramid_indices = {
            1,2,0,
            2,1,3,
            3,1,0,
            3,0,2
};

// four vertices of pyramid.
// each 3 float represent 1 vertices.
const std::array<float, 12> pyramid_vertices = {
        -2,0,0,
        2,0,0,
        0,2,0,
        0,0,2
};

using merely3d::renderable;
using merely3d::Rectangle;
using merely3d::Box;
using merely3d::Line;
using merely3d::Color;
using merely3d::Material;
using merely3d::Sphere;
using merely3d::red;
using merely3d::Particle;

using Eigen::AngleAxisf;
using Eigen::Vector3f;
using namespace Eigen;
using Simulator::RealVector3;

    class marching_cube_visual
    {
    public:
        marching_cube_visual():model(load_example_model()){

        };

        void render(merely3d::Frame &frame)
        {
            // Below we demonstrate how to render various primitives with merely3d.

            const auto floor_color = Color(0.9, 0.9, 0.9);

            frame.draw(renderable(Rectangle(20.0f, 20.0f))
                               .with_position(0.0f, 0.0f, 0.0f)
                               .with_material(Material().with_color(floor_color)));
;

            // Note that if you use RealVector3 for your position, you might need to cast it into a Vector3f
            // since merely3d uses float for positions. Here's an example:



            const auto model_color = Color(0.0, 0.6, 0.0);

            // Render a few instances of our example model. Note: It looks like the connectivity
            // of our model may be a little messed up, and as a result attempts to render the model
            // as a wireframe may give strange results.
            frame.draw(renderable(model)
                        .with_position(8.0, 8.0, 3.0)
                       .with_material(Material().with_pattern_grid_size(0).with_color(model_color))
                        );
            /// Begin begins a new ImGui window that you can move around as you please
            if (ImGui::Begin("Hello", NULL, ImVec2(300, 200)))
            {


            }
            ImGui::End();


        }

    private:
        merely3d::StaticMesh load_example_model()
        {
            // Create a static mesh which we can use when rendering
            const auto & vn = pyramid_vertices;
            const auto & idx = pyramid_indices;
            const auto model_vertices = std::vector<float>(vn.begin(), vn.end());
            const auto model_indices = std::vector<unsigned int>(idx.begin(), idx.end());
            return merely3d::StaticMesh::with_angle_weighted_normals(model_vertices, model_indices);
            //return merely3d::StaticMesh(model_vertices_normals, model_indices);
        }



        //------- private varaible
        merely3d::StaticMesh model;

        const float unit_voxel_length; // the resolution parameter.
    };



