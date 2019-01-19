#pragma once

#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>

#include "math_types.hpp"

#include <Eigen/Geometry>

using Eigen::Vector3f;

struct mMeshData
{
    std::vector<float> vertices_and_normals;
    std::vector<unsigned int> faces;
    Vector3f bounding_box;
    Vector3f origin;

    template <class Archive>
    void serialize( Archive & ar )
    {
    	ar( vertices_and_normals );
    	ar( faces );
    	ar( bounding_box[0], bounding_box[1], bounding_box[2] );
    	ar( origin[0], origin[1], origin[2] );
    }

};

typedef struct mMeshData mMeshData;

struct mMeshSeries
{
	std::vector<mMeshData> meshSeries;

	float unit_voxel_length;
    float c;

	template <class Archive>
    void serialize( Archive & ar )
    {
    	ar( meshSeries, unit_voxel_length, c);
    }
};

typedef struct mMeshSeries mMeshSeries;
