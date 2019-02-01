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

#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

using Simulator::Real;

using std::chrono::duration;
using std::cout;
using std::endl;
#include <array>


////////////////////////////////////////////////////
std::atomic<bool> quit(false);    // signal flag
void got_signal(int)
{
    quit.store(true);
}
/////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
    CLI::App CLIapp{"serialize fluid mesh using marching cube"};

    // Define options
    float unit_voxel_length = 0.1;
    CLIapp.add_option("-u, --unit_length", unit_voxel_length, "resolution parameter");

    const int skip = 1;

    double c = 0.6;
    CLIapp.add_option("-c", c, "estimated surface density");

    CLIapp.option_defaults()->required();

    std::string input_file;
    CLIapp.add_option("-i, --input_file", input_file, "path to input cerealed simulation file");

    std::string output_file;
    CLIapp.add_option("-o, --output_file", output_file, "path to output cerealed mesh file");

    try {
        CLIapp.parse(argc, argv);
    } catch(const CLI::ParseError &e) {
        return CLIapp.exit(e);
    }

	mMeshSeries ms;

    static vector<unsigned int> output_indices;
    static vector<float> output_vertices;

    marching_cube_fluid fluid(unit_voxel_length, c, input_file);
    fluid.start_marching_cube();

    fluid.output_marching_indices(output_indices);
    fluid.output_marching_vertices_and_normals(output_vertices);

    mMeshData md;
    md.vertices_and_normals = output_vertices;
    md.faces = output_indices;
    md.bounding_box = Vector3f(fluid.total_x_length, fluid.total_y_length, fluid.total_z_length);
    md.origin = fluid.origin;

    ms.meshSeries.push_back(md);

    ms.c = static_cast<float>(c);
    ms.unit_voxel_length = unit_voxel_length;

    fluid.count += skip;

    ////////////////////////////////////////////////////////////////
    // code allow we use ctrl-c to quit the simulation while it still store data.
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);
    //////////////////////////////////////////////////////////////////////////

    while (!fluid.end)
    {
        fluid.update_marching_cube();
        if (fluid.end)
        	break;

        fluid.output_marching_indices(output_indices);
        fluid.output_marching_vertices_and_normals(output_vertices);

        mMeshData md2;
        md2.vertices_and_normals = output_vertices;
        md2.faces = output_indices;
        md2.bounding_box = Vector3f(fluid.total_x_length, fluid.total_y_length, fluid.total_z_length);
        md2.origin = fluid.origin;

        ms.meshSeries.push_back(md2);

        fluid.count += skip;
        cout<<"iteration "<<fluid.count<<endl;
        ////////////////////////////////////////////////////////////
        if( quit.load() ) break;    // exit normally after SIGINT
        ////////////////////////////////////////////////////////////
    }

    std::ofstream os(output_file, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );

    archive( ms );
    cout<<"cerealing mesh data!"<<endl;
    return 0;
}
