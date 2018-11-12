#pragma once

#include "triangle_mesh.hpp"

#include <string>

namespace Simulator
{
    struct MeshPair
    {
        TriangleMesh mesh;
        RenderMesh   render_mesh;
        std::string  name;
    };

    struct ObjResult
    {
        bool success;

        std::vector<MeshPair>     mesh_results;

        // Contains errors (if success is false) and/or warnings emitted by the OBJ loader.
        std::string               message;
    };

    ObjResult load_obj(const std::string & filename);
}