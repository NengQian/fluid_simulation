#include <memory>
#include <iostream>
#include <algorithm>

#include <merely3d/app.hpp>
#include <merely3d/window.hpp>
#include <merely3d/camera_controller.hpp>

#include <Eigen/Geometry>
#include <CLI11.hpp>

#include <imgui/imgui.h>
#include <imgui/imgui_event_handler.h>

#include <chrono>

#include "math_types.hpp"
#include "marching_cube.hpp"
#include "marching_cube_torus.hpp"
#include "marching_cube_sphere.hpp"

using merely3d::Window;
using merely3d::WindowBuilder;
using merely3d::Frame;
using merely3d::Key;
using merely3d::Action;
using merely3d::EventHandler;
using merely3d::Material;
using merely3d::Color;
using merely3d::CameraController;
using merely3d::renderable;
using merely3d::Rectangle;
using merely3d::Box;
using merely3d::red;
using merely3d::Line;
using merely3d::Sphere;
using merely3d::Camera;
using merely3d::Particle;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

using Simulator::Real;

using std::chrono::duration;

#include <array>

// each three indices represent a triangle face
// here we have 6 faces
// note the vertices order of one triangle face should be counter-clock wise
const std::array<unsigned int, 18> pyramid_indices = {
            1,2,0,
			2,4,0,
			4,3,0,
			3,1,0,
			1,3,2,
			3,4,2
};

// four vertices of pyramid.
// each 3 float represent 1 vertices.
const std::array<float, 18> pyramid_vertices = {
        0,0,0,
        1,1,-2,
        -1,1,-2,
        1,-1,-2,
		-1,-1,-2
};


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

void load_scene(Camera & camera)
{
    // Set up the camera the way you want it
    camera.look_in(Vector3f(1.0, 0.0, -1), Vector3f(0.0, 0.0, 1.0));
    camera.set_position(Vector3f(-10.0, 0.0, 10.0));

    // Add bodies to your simulation
}


//void draw_grid(marching_cube_sphere& mcs,Frame & frame){
//    for(auto it = mcs.voxel_vertices.begin();it!=mcs.voxel_vertices.end();++it)
//    {
//        const auto model_color = Color(0.0, 0.0, 1.0);

//        frame.draw(renderable(Sphere(0.05)).with_position(it->position)
//     .with_material(Material().with_pattern_grid_size(0.025).with_color(model_color)));
//    }

//    for(auto it = mcs.edges_vector_x.begin();it!=mcs.edges_vector_x.end();++it)
//    {
//        const auto model_color = Color(1.0, 0.0, 0.0);
//        frame.draw_line(Line(it->vertex1_ptr->position,it->vertex2_ptr->position, model_color));
//    }

//    for(auto it = mcs.edges_vector_y.begin();it!=mcs.edges_vector_y.end();++it)
//    {
//        const auto model_color = Color(0.0, 1.0, 0.0);
//        frame.draw_line(Line(it->vertex1_ptr->position,it->vertex2_ptr->position,model_color));
//    }
//    for(auto it = mcs.edges_vector_z.begin();it!=mcs.edges_vector_z.end();++it)
//    {
//        const auto model_color = Color(0.0, 0.0, 1.0);
//        frame.draw_line(Line(it->vertex1_ptr->position,it->vertex2_ptr->position,model_color));
//    }

//    for(auto it = mcs.voxels.begin();it!=mcs.voxels.end();++it)
//    {
//        for(int i = 0; i<8;++i)
//        {
//            mVoxelvertex& vtx = *(it->vertex_ptrs[i]);
//            //const auto model_color =red;
//            const auto model_color = Color(1., 0.0, 0.0);
//            const auto green = Color(0.0, 1.0, 0.0);

//            if(vtx.is_inside)
//                frame.draw(renderable(Sphere(0.05)).with_position(vtx.position)
//             .with_material(Material().with_pattern_grid_size(0.025).with_color(model_color)));
//            else
//                frame.draw(renderable(Sphere(0.025)).with_position(vtx.position)
//             .with_material(Material().with_pattern_grid_size(0.025).with_color(green)));

//        }
//        for(int i = 0; i<12;++i)
//        {
//            mVoxel_edge& veg = *(it->voxel_edges_ptrs[i]);
//            //const auto model_color =red;
//            const auto model_color = Color(1., 0.0, 0.0);
//            const auto green = Color(0.0, 1.0, 0.0);
//            //if(veg.has_mesh_vertex)
//            {
//                frame.draw_line(Line(veg.vertex1_ptr->position,veg.vertex2_ptr->position));
//            }

//        }
//    }
//}

int main(int argc, char* argv[])
{
    CLI::App CLIapp{"test marching cube"};

    // Define options
    float unit_voxel_length = 0.1;
    CLIapp.add_option("-u, --unit_length", unit_voxel_length, "resolution parameter");
    try {
        CLIapp.parse(argc, argv);
    } catch(const CLI::ParseError &e) {
        return CLIapp.exit(e);
    }

    using Simulator::Real;
    merely3d::App app;

    auto window = WindowBuilder()
            .dimensions(1024, 768)
            .title("Simulation")
            .multisampling(8)
            .build();


    window.add_event_handler(std::shared_ptr<EventHandler>(new Simulator::ImGuiEventHandler));
    window.add_event_handler(std::shared_ptr<EventHandler>(new CameraController));



    load_scene( window.camera());

    merely3d::StaticMesh model(load_example_model());

    vector<unsigned int> output_indices;
    vector<float> output_vertices_and_normals;
    vector<float> output_vertices;
    {
        // marching_cube cost many memeory, so run this in the scope, when it leave the scope, the marching_cube instance
        // delete itself and release mememory.
        //////////////////////////////////////////////////////////////////////////////
        marching_cube_sphere sphere_cube(unit_voxel_length);
        sphere_cube.start_marching_cube();  // use together! now we need start functio to start computing
        //////////////////////////////////////////////////////////////////////////////
        sphere_cube.output_marching_indices(output_indices);
        sphere_cube.output_marching_vertices_and_normals(output_vertices_and_normals);
        sphere_cube.output_marching_vertices(output_vertices);
    }
    merely3d::StaticMesh sphere_model(output_vertices_and_normals,output_indices);
    merely3d::StaticMesh sphere_model_without_normal = merely3d::StaticMesh::with_angle_weighted_normals(output_vertices,output_indices);

    ///////////////////////////////////output the length of vertices and indices////////////////////////////////
    cout<<"sphere in "<<unit_voxel_length<<" resolution, indices's lenghth = "<<output_indices.size()
       <<"vertice length"<< output_vertices.size()<<endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    vector<unsigned int> output_indices_torus;
    vector<float> output_vertices_and_normals_torus;
    vector<float> output_vertices_torus;
    {
        marching_cube_torus torus_cube(unit_voxel_length);
        torus_cube.start_marching_cube();
        torus_cube.output_marching_indices(output_indices_torus);
        torus_cube.output_marching_vertices(output_vertices_torus);
        torus_cube.output_marching_vertices_and_normals(output_vertices_and_normals_torus);
    }
    merely3d::StaticMesh torus_model(output_vertices_and_normals_torus,output_indices_torus);
    merely3d::StaticMesh torus_model_without_normal = merely3d::StaticMesh::with_angle_weighted_normals(output_vertices_torus,output_indices_torus);




    while (!window.should_close())
    {
        window.render_frame([&] (Frame & frame)
        {
            const auto floor_color = Color(0.9, 0.9, 0.9);

            frame.draw(renderable(Rectangle(20.0f, 20.0f))
                               .with_position(0.0f, 0.0f, 0.0f)
                               .with_material(Material().with_color(floor_color)));

            const auto model_color = Color(1., 1., 0.0);
            const auto model_color2 = Color(0.0,0.6,0.6);
            // Render a few instances of our example model. Note: It looks like the connectivity
            // of our model may be a little messed up, and as a result attempts to render the model
            // as a wireframe may give strange results.

            frame.draw(renderable(model)
                        .with_position(-7.0, 0.0, 4.5)
                        );
            frame.draw(renderable(sphere_model)
                        .with_position(-1.0, 0.0, 3.0)
                        .with_material(Material().with_pattern_grid_size(0).with_color(model_color))
                        );

            frame.draw(renderable(sphere_model_without_normal)
                        .with_position(-4, 0.0, 3.0)
                        .with_material(Material().with_pattern_grid_size(0).with_color(model_color))
                        );
            //draw_grid(sphere_cube,frame);

            frame.draw(renderable(torus_model_without_normal)
                        .with_position(3.0, 0.0, 3.0)
                        .with_material(Material().with_pattern_grid_size(0).with_color(model_color2))
                        );


            frame.draw(renderable(torus_model)
                        .with_position(7.0, 0.0, 3.0)
                        .with_material(Material().with_pattern_grid_size(0).with_color(model_color2))
                        );
        });
    }

    return 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
}
