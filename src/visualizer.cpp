#include <memory>
#include <iostream>
#include <algorithm>
#include <chrono>

#include <merely3d/app.hpp>
#include <merely3d/window.hpp>
#include <merely3d/camera_controller.hpp>

#include <Eigen/Geometry>

#include <imgui/imgui.h>
#include <imgui/imgui_event_handler.h>

#include <CLI11.hpp>

#include "math_types.hpp"
#include "visual.hpp"

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

void load_scene(Camera & camera)
{
    // Set up the camera the way you want it
    camera.look_in(Vector3f(1.0, 0.0, -0.1), Vector3f(0.0, 0.0, 1.0));
    camera.set_position(Vector3f(-10.0, 0.0, 3.0));

    // Add bodies to your simulation
}

/// Simple helper class to manage time.
///
/// Whenever time passes in the real world, time is "produced".
/// In order to advance the simulation time, there must be
/// enough available time for consumption.
///
/// This approach allows you to decouple the physics time step
/// from your rendering updates. See
/// https://gafferongames.com/post/fix_your_timestep/
/// for more information.


int main(int argc, char* argv[])
{
	CLI::App CLIapp{"fluid visualizer"};

    

	std::vector<std::string> meshfiles;
	CLIapp.add_option("-m, --mesh", meshfiles, "path to serialized mesh data");

	CLIapp.option_defaults()->required();

	std::vector<std::string> simfiles;
	CLIapp.add_option("-s, --sim", simfiles, "path to serialized simulation data");

    std::vector<float> shift_x;
	CLIapp.add_option("-x", shift_x, "shifts of different sets of data along x (if you have only one set of simualtion (and mesh) data, then x=0 is enough)");

	try
	{
		CLIapp.parse(argc, argv);
	}
	catch (const CLI::ParseError &e)
	{
		return CLIapp.exit(e);
	}

    //
    if (shift_x.size() != simfiles.size())
    {
        std::cerr << "Please enter the same number of shifts and simulation files" << std::endl;
        return -1;
    }

    if (!meshfiles.empty() && meshfiles.size() != simfiles.size())
    {
        std::cerr << "Please enter the same number of simulation and mesh files" << std::endl;
        return -1;
    }

    // Constructing the app first is essential: it makes sure that
    // GLFW is set up properly. Note that as an alternative, you can call
    // glfw init/terminate yourself directly, but you must be careful that
    // any windows are destroyed before calling terminate(). App automatically
    // takes care of this as long as it outlives any windows.
    merely3d::App app;

    auto window = WindowBuilder()
            .dimensions(1366, 768)
            .title("Fluid Simulation")
            .multisampling(8)
            .build();

    // You can also add your own event handlers. See the EventHandler class for available events.
    // Note that the order of event handlers matter: They are called in the order they are added,
    // and for some events (like input events), each handler is able to stop further propagation of
    // events. Thus you probably want to have ImGuiEventHandler and CameraController
    // added before any of your own event handlers.
    window.add_event_handler(std::shared_ptr<EventHandler>(new Simulator::ImGuiEventHandler));
    window.add_event_handler(std::shared_ptr<EventHandler>(new CameraController));
    std::vector<Visualization> visualizations;
    
    if (meshfiles.empty())
    {
        for (int i=0; i<simfiles.size(); ++i)
        {
            visualizations.push_back( Visualization(simfiles[i], shift_x[i]) );
        }
    }  else{
        for (int i=0; i<simfiles.size(); ++i)
        {
            visualizations.push_back( Visualization(simfiles[i], meshfiles[i], shift_x[i]) );
        }
    }

    // Here we currently only load a single scene at startup,
    // but you probably want to be able to dynamically reload different
    // scenes through your GUI.
    load_scene(window.camera());

    // You might want to make this configurable through your GUI!

    while (!window.should_close())
    {
        window.render_frame([&] (Frame & frame)
        {
            visualizations[0].render_control_panel();
            for (auto& vs : visualizations)
                vs.render(frame);
        });
    }

    return 0;
}
