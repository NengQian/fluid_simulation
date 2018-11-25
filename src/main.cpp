#include <merely3d/app.hpp>
#include <merely3d/window.hpp>
#include <merely3d/camera_controller.hpp>

#include <Eigen/Geometry>

#include <imgui/imgui.h>
#include <imgui/imgui_event_handler.h>

#include <CLI11.hpp>
#include <iostream>

#include <chrono>

#include "math_types.hpp"
#include "simulation.hpp"

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

void load_scene(Simulator::Simulation & sim, Camera & camera)
{
    // Set up the camera the way you want it
    camera.look_in(Vector3f(1.0, 0.0, -1), Vector3f(0.0, 0.0, 1.0));
    camera.set_position(Vector3f(-1.0, 0.0, 3.0));

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
class TimeKeeper
{
public:
    TimeKeeper()
            : _available_time(0.0)
    {}

    bool consume(double dt)
    {
        if (dt > 0.0 && _available_time >= dt)
        {
            _available_time -= dt;
            return true;
        }
        else
        {
            return false;
        }
    }

    void produce(double dt)
    {
        _available_time += dt;
    }

private:
    double _available_time;
};

int main(int argc, char **argv)
{
    using Simulator::Simulation;
    using Simulator::Real;

    // Constructing the app first is essential: it makes sure that
    // GLFW is set up properly. Note that as an alternative, you can call
    // glfw init/terminate yourself directly, but you must be careful that
    // any windows are destroyed before calling terminate(). App automatically
    // takes care of this as long as it outlives any windows.
    merely3d::App app;

    auto window = WindowBuilder()
            .dimensions(1024, 768)
            .title("Simulation")
            .multisampling(8)
            .build();

    // You can also add your own event handlers. See the EventHandler class for available events.
    // Note that the order of event handlers matter: They are called in the order they are added,
    // and for some events (like input events), each handler is able to stop further propagation of
    // events. Thus you probably want to have ImGuiEventHandler and CameraController
    // added before any of your own event handlers.
    window.add_event_handler(std::shared_ptr<EventHandler>(new Simulator::ImGuiEventHandler));
    window.add_event_handler(std::shared_ptr<EventHandler>(new CameraController));

//<<<<<<< HEAD

    // Use CLI to parse the command line arguments
    CLI::App CLIapp{"simulator"};

    // Define options
    float eta = 1.2f;
    CLIapp.add_option("-e, --eta", eta, "Eta: normally 1.0~1.5");

    float rest_density = 1000.0f;
    CLIapp.add_option("-d, --rest_density", rest_density, "Fluid Rest density: 1000 (kg/m^3) on water for instance");

    float B = 1000.0f;
    CLIapp.add_option("-s, --stiffness", B, "Stiffness of pressure force");

    float dt = 0.01f;
    CLIapp.add_option("-t, --dt", dt, "Elapsed time");

    std::vector<float> cuboid_side_lengths = {2.0f};
    CLIapp.add_option("-l, --side_lengths", cuboid_side_lengths, "Side length of cuboid(if existed)");

    float particle_radius = 0.1f;
    CLIapp.add_option("-r, --particle_radius", particle_radius, "Radius of particles");

    float alpha = 0.08f;
    CLIapp.add_option("-a, --alpha", alpha, "parameter of viscosity");

    CLIapp.option_defaults()->required();
    int N = 3;
    CLIapp.add_option("-n, --N", N, "Number of particles per edge");

    int mode = 1;
    CLIapp.add_option("-m, --mode", mode, "Simulation mode: 1 for rigid_body | 2 for free fall | 3 for 2-cube collision");

    CLI11_PARSE(CLIapp, argc, argv);

    Simulation simulation(N, mode, static_cast<Real>(dt), static_cast<Real>(eta), static_cast<Real>(B), static_cast<Real>(alpha), static_cast<Real>(rest_density));
//=======
//    const auto dt = 0.01;
//    Simulation simulation(dt, 1,3);
//>>>>>>> neng3

    // Here we currently only load a single scene at startup,
    // but you probably want to be able to dynamically reload different
    // scenes through your GUI.
    load_scene(simulation, window.camera());

    // You might want to make this configurable through your GUI!

    TimeKeeper keeper;

    auto last_frame_begin = std::chrono::steady_clock::now();

    while (!window.should_close())
    {
        // Produce real time that the simulation time can consume
        const auto now = std::chrono::steady_clock::now();
        const duration<double> elapsed = now - last_frame_begin;
        last_frame_begin = now;
        keeper.produce(elapsed.count());

        while (keeper.consume(dt))
        {
            simulation.timestep(dt);
        }
        //
        //if(simulation.is_simulation_finshed())
        //{
        //    std::cout<<"simulation finished!"<<std::endl;
        //    break;
        //}
        simulation.update();

        window.render_frame([&] (Frame & frame)
        {
            // Render the current state of your simulation.
            simulation.render(frame);
        });
    }

    return 0;
}
