#pragma once

#include <array>
#include <vector>
#include <string>

#include "../math_types.hpp"

namespace Simulator
{
    typedef std::array<int, 3> MeshFace;

    /**
     * A straightforward representation of a triangle mesh.
     */
    struct TriangleMesh
    {
        /// A list of all vertices in the mesh.
        std::vector<RealVector3>        vertices;

        /// A list of all faces in the mesh. Each face is a 3-element array of unsigned integers, each integer
        /// representing the index of a vertex (and corresponding normal), such that together the three
        /// vertices make up a triangle. Note that the indices/vertices must be listed in counter-clockwise winding
        /// order.
        std::vector<MeshFace>           faces;
    };

    /// Like TriangleMesh, but with additional normal data for rendering purposes.
    ///
    /// The reason for the additional type is that the render mesh may sometimes hold duplicate vertex
    /// data to ensure that sharp edges are correctly lit and not smoothed out in the lighting calculations.
    struct RenderMesh
    {
        /// A list of all vertices in the mesh.
        std::vector<RealVector3>        vertices;

        /// A list of all normals in the mesh, with each normal corresponding to the vertex in the same position in
        /// the respective vertex list. That is, the number of normals must always be the same as the number of
        /// vertices.
        std::vector<RealVector3>        normals;


        /// A list of all faces in the mesh
        std::vector<MeshFace>           faces;
    };
}
