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

#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>


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

struct mMeshData
{
    std::vector<float> vertices_and_normals;
    std::vector<unsigned int> faces;

    template <class Archive>
    void serialize( Archive & ar )
    {
    	ar( vertices_and_normals );
    	ar( faces );
    }

};

typedef struct mMeshData mMeshData;

struct mMeshSeries
{
	std::vector<mMeshData> meshSeries;

	template <class Archive>
    void serialize( Archive & ar )
    {
    	ar( meshSeries );
    }

};

typedef struct mMeshSeries mMeshSeries;

void load_scene(Camera & camera)
{
    // Set up the camera the way you want it
    camera.look_in(Vector3f(1.0, 0.0, -1), Vector3f(0.0, 0.0, 1.0));
    camera.set_position(Vector3f(-5.0, 0.0, 10.0));

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

    // Check the number of parameters
   if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " NAME" << std::endl;
        return 1;
    }
    // Print the user's name:


    using Simulator::Visualization;
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
    std::string file(argv[1]);

    mMeshSeries ms;

    std::ifstream is(file);
    cereal::BinaryInputArchive infile(is);
    infile(ms);

    // Here we currently only load a single scene at startup,
    // but you probably want to be able to dynamically reload different
    // scenes through your GUI.
    load_scene(window.camera());

    int total_frame = ms.meshSeries.size();
    int counter = 0;
    bool pause = true;
    bool play_back = false;

    // You might want to make this configurable through your GUI!
    while (!window.should_close())
    {
    	auto md = ms.meshSeries[counter];

    	merely3d::StaticMesh mesh(md.vertices_and_normals, md.faces);

		window.render_frame([&] (Frame & frame)
		{
			// Render the current state of your simulation.
			const auto floor_color = Color(0.9, 0.9, 0.9);

			frame.draw(renderable(Rectangle(20.0f, 20.0f))
							   .with_position(0.0f, 0.0f, 0.0f)
							   .with_material(Material().with_color(floor_color)));

			const auto model_color = Color(0., 0., 0.6);
			// Render a few instances of our example model. Note: It looks like the connectivity
			// of our model may be a little messed up, and as a result attempts to render the model
			// as a wireframe may give strange results.

			frame.draw(renderable(mesh)
					   .with_position(0.0, 0.0, 3.0)
					   .with_material(Material().with_pattern_grid_size(0).with_color(model_color))
					  );


	        /// Begin begins a new ImGui window that you can move around as you please
			if (ImGui::Begin("Panel", NULL, ImVec2(300, 200)))
			{
				// See the code for ImGui::ShowDemoWinxdow() for examples of how to use
				// various ImGui widgets.
				ImGui::SliderInt("current frame", &counter, 0, total_frame-1);
				ImGui::Checkbox("play back", &play_back);
				ImGui::Checkbox("pause", &pause);
			}
			ImGui::End();
		});

    	if (!pause)
    	{
    		if (play_back)
    		{
    			--counter;
    			counter = std::max(counter, 0);
    		}
    		else
    		{
    			++counter;
    			counter = std::min(counter, total_frame-1);
    		}
    	}
    }
    /*
    for (auto& md : ms.meshSeries)
    {
    	merely3d::StaticMesh mesh(md.vertices_and_normals, md.faces);

        window.render_frame([&] (Frame & frame)
        {
            // Render the current state of your simulation.
            const auto floor_color = Color(0.9, 0.9, 0.9);

            frame.draw(renderable(Rectangle(20.0f, 20.0f))
                               .with_position(0.0f, 0.0f, 0.0f)
                               .with_material(Material().with_color(floor_color)));

            const auto model_color = Color(0., 0., 0.6);
            // Render a few instances of our example model. Note: It looks like the connectivity
            // of our model may be a little messed up, and as a result attempts to render the model
            // as a wireframe may give strange results.

            frame.draw(renderable(mesh)
                       .with_position(0.0, 0.0, 3.0)
                       .with_material(Material().with_pattern_grid_size(0).with_color(model_color))
                      );

        });


    }
    */

    return 0;
}
