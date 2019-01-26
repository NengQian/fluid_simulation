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
#include "sim_record.hpp"

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

std::vector<double> get_boundary_size(std::vector<mParticle>& boundary_particles, float unit_voxel_length)
{
    double max_x = INT_MIN;
    double max_y = INT_MIN;
    double max_z = INT_MIN;

    double min_x = INT_MAX;
    double min_y = INT_MAX;
    double min_z = INT_MAX;

    for (auto& p : boundary_particles)
    {
        if (p.position[0] > max_x)
            max_x = p.position[0];
        if (p.position[0] < min_x)
            min_x = p.position[0];

        if (p.position[1] > max_y)
            max_y = p.position[1];
        if (p.position[1] < min_y)
            min_y = p.position[1];

        if (p.position[2] > max_z)
            max_z = p.position[2];
        if (p.position[2] < min_z)
            min_z = p.position[2];
    }

    double du = static_cast<double>(unit_voxel_length);

    double max_x_unit = (ceil(max_x/du*0.5) + 1) * 2.0 * du;
    double min_x_unit = (floor(min_x/du*0.5) - 1) * 2.0 * du;

    double max_y_unit = (ceil(max_y/du*0.5) + 1) * 2.0 * du;
    double min_y_unit = (floor(min_y/du*0.5) - 1) * 2.0 * du;

    double max_z_unit = (ceil(max_z/du*0.5) + 1) * 2.0 * du;
    double min_z_unit = (floor(min_z/du*0.5) - 1) * 2.0 * du;

    // update min, max
    min_x = min_x_unit;
    min_y = min_y_unit;
    min_z = min_z_unit;

    max_x = max_x_unit;
    max_y = max_y_unit;
    max_z = max_z_unit;

    std::vector<double> bounding_box;
    bounding_box.push_back(min_x);
    bounding_box.push_back(max_x);
    bounding_box.push_back(min_y);
    bounding_box.push_back(max_y);
    bounding_box.push_back(min_z);

    return bounding_box;
    /*
    total_x_length = static_cast<float>(max_x_unit-min_x_unit);
    total_y_length = static_cast<float>(max_y_unit-min_y_unit);
    total_z_length = static_cast<float>(max_z_unit-min_z_unit);

    origin = Vector3f(static_cast<float>(max_x_unit+min_x_unit)*0.5f, static_cast<float>(max_y_unit+min_y_unit)*0.5f, static_cast<float>(min_z_unit));
    */
}

int main(int argc, char* argv[])
{
    CLI::App CLIapp{"output dynamic/fixed bounding box info"};

    // Define options
    float unit_voxel_length = 0.1;
    CLIapp.add_option("-u, --unit_length", unit_voxel_length, "resolution parameter");

    bool fixed_box;
    CLIapp.add_flag("-f", fixed_box, "use static bounding box instead");

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

    float fix_box_x, fix_box_y, fix_box_z;
    std::vector<double> bounding_box;
    std::vector<mParticle> boundary_particles;
    
    if (fixed_box)
    {
        Visualization simData(input_file);
        boundary_particles = simData.sim_rec.boundary_particles;

        bounding_box = get_boundary_size(boundary_particles, unit_voxel_length);
    }

    marching_cube_fluid fluid(unit_voxel_length, c, input_file);
    int total_grids = fluid.voxelx_n * fluid.voxely_n * fluid.voxelz_n;
    
    if (fixed_box)
    {
        fluid.total_x_length = static_cast<float>(bounding_box[1]-bounding_box[0]);
        fluid.total_y_length = static_cast<float>(bounding_box[3]-bounding_box[2]);

        fluid.origin[0] = static_cast<float>((bounding_box[1]+bounding_box[0])*0.5f);
        fluid.origin[1] = static_cast<float>((bounding_box[3]+bounding_box[2])*0.5f);

        fluid.update_voxel();
        total_grids = fluid.voxelx_n * fluid.voxely_n * fluid.voxelz_n;
    }

    fluid.pick_up_particles();
    int total_lost = fluid.total_lost;

    fluid.count += skip;

    while (!fluid.end)
    {
        std::cout << total_grids << " " << total_lost << std::endl;
        fluid.load_next_particles();
    	if (fluid.end)
    		break;

		if (!fixed_box)
        {
            fluid.update_grid_size();
		    fluid.pick_up_particles();
        } else {
            fluid.count_exclusive_particles_in_fixed_box();
        }
        total_grids = fluid.voxelx_n * fluid.voxely_n * fluid.voxelz_n;
        total_lost = fluid.total_lost;

        fluid.count += skip;
    }

    return 0;
}
