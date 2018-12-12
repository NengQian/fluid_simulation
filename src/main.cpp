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
using std::cout;
using std::endl;

void load_scene(Simulator::Simulation & sim, Camera & camera)
{
    // Set up the camera the way you want it
    camera.look_in(Vector3f(1.0, 0.0, -1), Vector3f(0.0, 0.0, 1.0));
    camera.set_position(Vector3f(-1.0, 0.0, 1.0));

    // Add bodies to your simulation
}


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

    float alpha = 0.08f;
    CLIapp.add_option("-a, --alpha", alpha, "parameter of viscosity");

    std::string output_file = "../../sim_result/test.bin";
    CLIapp.add_option("-o, --output_file", output_file, "output filename");

    bool if_print_iteration = false;
    CLIapp.add_option("-p, --log", if_print_iteration, "decide if print iteration count, 0 for unprint or 1 for print");

    int with_viscosity = 1;
    CLIapp.add_option("-v, --with_viscosity", with_viscosity, "add viscosity");

    int with_XSPH = 1;
    CLIapp.add_option("-x, --with_XSPH", with_XSPH, "use XSPH to update position");

    CLIapp.option_defaults()->required();
    int N = 3;
    CLIapp.add_option("-n, --N", N, "Number of particles per edge");

    int mode = 1;
    CLIapp.add_option("-m, --mode", mode, "Simulation mode: 1 for dam breaking | 2 for dropping the water from the center of boundary | 3 for free fall | 4 for 2-cube collision");

    float unit_particle_length ;
    CLIapp.add_option("-u, --unit_particle_length", unit_particle_length, " the intervel length between two particles per axis.");

    CLI11_PARSE(CLIapp, argc, argv);
    cout<<"Eta = "<<eta<<endl;
    cout<<"rest_density = "<< rest_density <<endl;
    cout<<"stiffness = "<< B <<endl;
    cout<<"elapsed time = "<< dt <<endl;
    cout<<"alpha = "<<alpha<<endl;
    cout<<"with_viscosity = "<< with_viscosity<<endl;
    cout<<"with_XSPH = "<<with_XSPH<<endl;
    cout<<"particles number per edge = "<< N <<endl;
    cout<<"mode = "<< mode<< endl;
    cout<<"unit_particle_length = "<< unit_particle_length<<endl;
    cout<<"output_file = "<< output_file<<endl;
    cout<<"print iteration = "<< if_print_iteration<<endl;
    cout<<endl;

    Simulation simulation(N, mode, unit_particle_length, static_cast<Real>(dt), static_cast<Real>(eta), static_cast<Real>(B), static_cast<Real>(alpha), static_cast<Real>(rest_density),output_file,if_print_iteration, with_viscosity, with_XSPH);


    // Here we currently only load a single scene at startup,
    // but you probably want to be able to dynamically reload different
    // scenes through your GUI.
    load_scene(simulation, window.camera());

    // You might want to make this configurable through your GUI!



    while (!window.should_close())
    {
        simulation.update();
        window.render_frame([&] (Frame & frame)
        {
            // Render the current state of your simulation.
            simulation.render(frame);
        });
    }

    return 0;
}
