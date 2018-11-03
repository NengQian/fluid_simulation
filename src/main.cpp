#include <memory>
#include <iostream>
#include <algorithm>

#include <merely3d/app.hpp>
#include <merely3d/window.hpp>
#include <merely3d/camera_controller.hpp>

#include <Eigen/Geometry>

#include <imgui/imgui.h>
#include <imgui/imgui_event_handler.h>

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

int main()
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

    Simulation simulation(0.5f);

    // Here we currently only load a single scene at startup,
    // but you probably want to be able to dynamically reload different
    // scenes through your GUI.
    load_scene(simulation, window.camera());

    // You might want to make this configurable through your GUI!
    const auto dt = 0.01;

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

        window.render_frame([&] (Frame & frame)
        {
            // Render the current state of your simulation.
            simulation.render(frame);
        });
    }

    return 0;
}
