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

#include <time.h>

#include "math_types.hpp"
#include "marching_cube_fluid.hpp"
#include "marching_cube.hpp"


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

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

using Simulator::Real;

using std::chrono::duration;

#include <array>

void load_scene(Camera & camera)
{
    // Set up the camera the way you want it
    camera.look_in(Vector3f(1.0, 0.0, -1), Vector3f(0.0, 0.0, 1.0));
    camera.set_position(Vector3f(-3.0, 0.0, 5.0));

    // Add bodies to your simulation
}

int main(int argc, char* argv[])
{
    CLI::App CLIapp{"render fluid using marching cube"};

    // Define options
    float unit_voxel_length = 0.1;
    CLIapp.add_option("-u, --unit_length", unit_voxel_length, "resolution parameter");

    CLIapp.option_defaults()->required();

    std::string input_file;
    CLIapp.add_option("-i, --input_file", input_file, "path to input cereal file");

    double c;
    CLIapp.add_option("-c", c, "estimated surface density");

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



    load_scene( window.camera() );

    static vector<unsigned int> output_indices;
    static vector<float> output_vertices;

    marching_cube_fluid fluid(unit_voxel_length, c, input_file);
    fluid.start_marching_cube();

    fluid.output_marching_indices(output_indices);
    fluid.output_marching_vertices_and_normals(output_vertices);

    merely3d::StaticMesh fluid_model_without_normal(output_vertices,output_indices);

    // main loop
    clock_t runtime = 0;
    clock_t lastTime = clock();
    clock_t fpsTime = lastTime;
    auto fpsCnt = 0.0;
    do {
        auto newTime = clock();
        runtime += newTime - lastTime;
        lastTime = newTime;

        // fps
        ++fpsCnt;
        if (lastTime > fpsTime + 2) {
            float fps = fpsCnt / (lastTime - fpsTime) * CLOCKS_PER_SEC;
            std::cout << "FPS: " << fps << std::endl;
            fpsTime = lastTime;
            fpsCnt = 0;
        }

        // actual rendering
        window.render_frame([&] (Frame & frame)
        {
            const auto floor_color = Color(0.9, 0.9, 0.9);

            frame.draw(renderable(Rectangle(20.0f, 20.0f))
                               .with_position(0.0f, 0.0f, 0.0f)
                               .with_material(Material().with_color(floor_color)));

            const auto model_color = Color(0., 0., 0.6);
            // Render a few instances of our example model. Note: It looks like the connectivity
            // of our model may be a little messed up, and as a result attempts to render the model
            // as a wireframe may give strange results.

            frame.draw(renderable(fluid_model_without_normal)
                       .with_position(0.0, 0.0, 3.0)
                       .with_material(Material().with_pattern_grid_size(0).with_color(model_color))
                      );
        });

        fluid.update_marching_cube();

        fluid.output_marching_indices(output_indices);
        fluid.output_marching_vertices_and_normals(output_vertices);

        merely3d::StaticMesh fluid_model_without_normal(output_vertices,output_indices);

    } while (!window.should_close());

    return 0;
}
