#include "obj.hpp"

#define TINYOBJLOADER_USE_DOUBLE
#define TINYOBJLOADER_IMPLEMENTATION
#include <tinyobjloader/tiny_obj_loader.h>

#include <cassert>
#include <unordered_map>
#include <set>

namespace Simulator
{
    struct VertexRemap
    {
        std::vector<RealVector3> vertices;
        std::unordered_map<int, int> vertex_index_map;
    };

    /// Determines which vertices are actually referenced in the given mesh,
    /// returning a (possibly reduced) list of vertices and a mapping from original indices
    /// to indices in the list of vertices.
    static VertexRemap remap_vertices(const tinyobj::attrib_t & attrib, const tinyobj::mesh_t & mesh)
    {
        assert(attrib.vertices.size() % 3 == 0);

        VertexRemap result;

        const auto num_file_vertices = attrib.vertices.size() / 3;
        const auto num_file_faces = mesh.num_face_vertices.size();
        std::vector<bool> is_referenced(num_file_vertices, false);
        result.vertices.reserve(num_file_vertices);

        for (size_t i = 0; i < num_file_faces; ++i)
        {
            // Mesh should have been triangulated before calling this function!
            const auto num_vertices_in_face = mesh.num_face_vertices[i];
            assert(num_vertices_in_face == 3);

            const auto & idx = mesh.indices;
            const auto vertex_indices = MeshFace { idx[3 * i].vertex_index,
                                                   idx[3 * i + 1].vertex_index,
                                                   idx[3 * i + 2].vertex_index };

            for (const auto & index : vertex_indices)
            {
                // Invalid index shouldn't be possible because tinyobjloader should handle that for us
                assert(index >= 0 && index < num_file_vertices);
                is_referenced.at(static_cast<size_t>(index)) = true;
            }
        }

        int counter = 0;

        for (size_t i = 0; i < num_file_vertices; ++i)
        {
            if (is_referenced.at(i))
            {
                const auto & verts = attrib.vertices;
                const RealVector3 v { static_cast<Real>(verts[3 * i]),
                                      static_cast<Real>(verts[3 * i + 1]),
                                      static_cast<Real>(verts[3 * i + 2]) };
                result.vertices.push_back(v);
                result.vertex_index_map[i] = counter;
                ++counter;
            }
        }

        result.vertices.shrink_to_fit();
        return result;
    };

    struct VertexNormalRemap
    {
        std::vector<RealVector3> vertices;
        std::vector<RealVector3> normals;

        // Maps a pair of (file_vertex_index, file_normal_index) to a remapped vertex index
        // (i.e. an index in the 'vertices' vector)
        std::map<std::pair<int, int>, int> vertex_normal_index_map;
    };

    /// Determines which (vertex, normal) pairs are actually referenced in the given mesh,
    /// returning a (possibly reduced) list of (possibly duplicated) vertices and a mapping from original indices
    /// to indices in the list of vertices.
    static VertexNormalRemap remap_vertices_and_normals(const tinyobj::attrib_t & attrib, const tinyobj::mesh_t & mesh)
    {
        assert(attrib.vertices.size() % 3 == 0);

        VertexRemap result;

        const auto num_file_faces = mesh.num_face_vertices.size();

        std::set<std::pair<int, int>> present_vertex_normal_pairs;


        for (size_t i = 0; i < num_file_faces; ++i)
        {
            const auto num_vertices_in_face = mesh.num_face_vertices[i];
            assert(num_vertices_in_face == 3);

            for (size_t j = 0; j < 3; ++j)
            {
                const auto file_index = mesh.indices[3 * i + j];
                present_vertex_normal_pairs.insert(std::make_pair(file_index.vertex_index, file_index.normal_index));
            }
        }

        VertexNormalRemap remap;
        remap.vertices.reserve(present_vertex_normal_pairs.size());
        remap.normals.resize(present_vertex_normal_pairs.size(), RealVector3::Zero());

        int counter = 0;

        for (const auto & pair : present_vertex_normal_pairs)
        {
            const auto file_vertex_index = pair.first;
            const auto file_normal_index = pair.second;
            const auto & verts = attrib.vertices;
            const auto & norms = attrib.normals;
            const RealVector3 v { static_cast<Real>(verts[3 * file_vertex_index]),
                                  static_cast<Real>(verts[3 * file_vertex_index + 1]),
                                  static_cast<Real>(verts[3 * file_vertex_index + 2]) };

            // We'll have to handle missing normals later
            if (file_normal_index != -1)
            {
                const RealVector3 n { static_cast<Real>(norms[3 * file_normal_index]),
                                      static_cast<Real>(norms[3 * file_normal_index + 1]),
                                      static_cast<Real>(norms[3 * file_normal_index + 2]) };
                remap.normals.at(counter) = n;
            }

            remap.vertices.push_back(v);
            remap.vertex_normal_index_map[pair] = counter;
            ++counter;
        }

        return remap;
    };

    static TriangleMesh extract_triangle_mesh(const tinyobj::attrib_t & attrib, const tinyobj::mesh_t & mesh)
    {
        assert(attrib.vertices.size() % 3 == 0);

        const auto num_file_faces = mesh.num_face_vertices.size();

        auto remapped = remap_vertices(attrib, mesh);

        std::vector<MeshFace> faces;

        for (size_t i = 0; i < num_file_faces; ++i)
        {
            const auto & idx = mesh.indices;
            const auto file_vertex_indices = MeshFace { idx[3 * i].vertex_index,
                                                        idx[3 * i + 1].vertex_index,
                                                        idx[3 * i + 2].vertex_index };
            const auto remapped_vertex_indices = MeshFace { remapped.vertex_index_map[file_vertex_indices[0]],
                                                            remapped.vertex_index_map[file_vertex_indices[1]],
                                                            remapped.vertex_index_map[file_vertex_indices[2]] };

            faces.push_back(remapped_vertex_indices);
        }

        TriangleMesh tri_mesh;
        tri_mesh.vertices = std::move(remapped.vertices);
        tri_mesh.faces = std::move(faces);
        return tri_mesh;
    }

    static RenderMesh extract_render_mesh(const tinyobj::attrib_t & attrib, const tinyobj::mesh_t & mesh)
    {
        auto remapped = remap_vertices_and_normals(attrib, mesh);

        const auto num_file_faces = mesh.num_face_vertices.size();

        std::vector<MeshFace> faces;

        for (size_t i = 0; i < num_file_faces; ++i)
        {
            MeshFace remapped_face;
            const auto num_vertices_in_face = mesh.num_face_vertices[i];
            assert(num_vertices_in_face == 3);

            for (size_t j = 0; j < 3; ++j)
            {
                const auto file_index = mesh.indices[3 * i + j];

                const auto remapped_vertex_index = remapped.vertex_normal_index_map.at(
                        std::make_pair(file_index.vertex_index, file_index.normal_index));

                remapped_face[j] = remapped_vertex_index;
            }

            const auto & a = remapped.vertices.at(static_cast<size_t>(remapped_face[0]));
            const auto & b = remapped.vertices.at(static_cast<size_t>(remapped_face[1]));
            const auto & c = remapped.vertices.at(static_cast<size_t>(remapped_face[2]));
            const RealVector3 area_vector = (b - a).cross(c - a);

            for (size_t j = 0; j < 3; ++j)
            {
                const auto file_normal_index = mesh.indices[3 * i + j].normal_index;

                // There is no normal vector assigned to this vertex, so let's add the contribution from the
                // face normal (weighted by area)
                if (file_normal_index == -1)
                {
                    const auto remapped_vertex_index = remapped_face[j];
                    remapped.normals.at(remapped_vertex_index) += area_vector;
                }
            }

            faces.push_back(remapped_face);
        }

        // Normalize all normals, in case we have computed some area-weighted normals, and also to avoid problems
        // with possibly inexact input (i.e. not precisely normalized in the precision we have available)
        for (auto & normal : remapped.normals)
        {
            normal.normalize();
        }

        RenderMesh result;
        result.vertices = std::move(remapped.vertices);
        result.normals = std::move(remapped.normals);
        result.faces = std::move(faces);
        return result;
    }

    ObjResult load_obj(const std::string & filename)
    {
        ObjResult result;

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string err;

        // Note: Make sure we triangulate the mesh
        result.success = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str(), nullptr, true);
        result.message = err;

        for (const auto & shape : shapes)
        {
            MeshPair mesh_result;
            mesh_result.mesh = extract_triangle_mesh(attrib, shape.mesh);
            mesh_result.render_mesh = extract_render_mesh(attrib, shape.mesh);
            result.mesh_results.emplace_back(std::move(mesh_result)
            );
        }

        return result;
    }
}
