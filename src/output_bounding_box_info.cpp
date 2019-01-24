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
#include "mesh_record.hpp"

#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

using Simulator::Real;

using std::chrono::duration;

#include <array>

int main(int argc, char* argv[])
{
    CLI::App CLIapp{"serialize fluid mesh using marching cube"};

    // Define options
    float unit_voxel_length = 0.1;
    CLIapp.add_option("-u, --unit_length", unit_voxel_length, "resolution parameter");

    const int skip = 1;

    CLIapp.option_defaults()->required();

    std::string input_file;
    CLIapp.add_option("-i, --input_file", input_file, "path to input cerealed simulation file");

    double c = 0.6;

    try {
        CLIapp.parse(argc, argv);
    } catch(const CLI::ParseError &e) {
        return CLIapp.exit(e);
    }

	mMeshSeries ms;

    static vector<unsigned int> output_indices;
    static vector<float> output_vertices;

    marching_cube_fluid fluid(unit_voxel_length, c, input_file);
    int total_grids = fluid.voxelx_n * fluid.voxely_n * fluid.voxelz_n;
    
    fluid.pick_up_particles();
    int total_lost = fluid.total_lost;

    fluid.count += skip;

    while (!fluid.end)
    {
        std::cout << total_grids << " " << total_lost << std::endl;
        fluid.load_next_particles();
    	if (fluid.end)
    		break;

		fluid.update_grid_size();
		fluid.pick_up_particles();

        total_grids = fluid.voxelx_n * fluid.voxely_n * fluid.voxelz_n;
        total_lost = fluid.total_lost;

        fluid.count += skip;
    }

    return 0;
}
