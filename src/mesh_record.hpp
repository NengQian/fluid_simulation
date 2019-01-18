#pragma once

#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include "math_types.hpp"

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
