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

int main(int argc, char* argv[])
{
    CLI::App CLIapp{"serialize fluid mesh using marching cube"};

    // Define options
    float unit_voxel_length = 0.1;
    CLIapp.add_option("-u, --unit_length", unit_voxel_length, "resolution parameter");

    int skip = 5;
    CLIapp.add_option("-z, --step_size", skip, "record once every <skip> frames");

    CLIapp.option_defaults()->required();

    std::string input_file;
    CLIapp.add_option("-i, --input_file", input_file, "path to input cerealed simulation file");

    std::string output_file;
    CLIapp.add_option("-o, --output_file", output_file, "path to output cerealed mesh file");

    double c;
    CLIapp.add_option("-c", c, "estimated surface density");

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
    fluid.output_marching_vertices(output_vertices);

    merely3d::StaticMesh fluid_model_without_normal = merely3d::StaticMesh::with_angle_weighted_normals(output_vertices,output_indices);


    mMeshData md;
    md.vertices_and_normals = fluid_model_without_normal._data->vertices_and_normals;
    md.faces = fluid_model_without_normal._data->faces;

    ms.meshSeries.push_back(md);

    fluid.count += skip;

    while (!fluid.end)
    {
        fluid.update_marching_cube();

        fluid.output_marching_indices(output_indices);
        fluid.output_marching_vertices(output_vertices);

        fluid_model_without_normal = merely3d::StaticMesh::with_angle_weighted_normals(output_vertices,output_indices);

        mMeshData md2;
        md2.vertices_and_normals = fluid_model_without_normal._data->vertices_and_normals;
        md2.faces = fluid_model_without_normal._data->faces;

        ms.meshSeries.push_back(md2);

        fluid.count += skip;
    }

    std::ofstream os(output_file, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );

    archive( ms );

    return 0;
}
