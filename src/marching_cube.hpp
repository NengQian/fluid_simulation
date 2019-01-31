#ifndef MARCHING_CUBE_H
#define MARCHING_CUBE_H
#include <vector>
#include <array>
#include <list>
#include <Eigen/Geometry>

using Eigen::Vector3f;

using namespace std;


struct mMeshvertex{
    Vector3f position;
    Vector3f normal;
    unsigned int id;
    mMeshvertex(Vector3f& init_pos, Vector3f& init_norm, unsigned int v_id):position(init_pos),normal(init_norm),id(v_id){}
};
typedef struct mMeshvertex mMesh_vertex;


struct mMeshtriangle{
    std::array<unsigned int,3> vertex_ids;
};
typedef struct mMeshtriangle mMesh_triangle;



struct mVoxelvertex{
    Vector3f position; // position of this vertex
    double phi;  // its phi value, which describle the distance to the surface
    bool is_inside;
};
typedef struct mVoxelvertex mVoxel_vertex;


struct mVoxelEdge{// each edge has 2 vertices
    mVoxel_vertex * vertex1_ptr;
    mVoxel_vertex * vertex2_ptr;
    bool has_mesh_vertex;
    mMesh_vertex * meshVertexptr;

    mVoxelEdge():has_mesh_vertex(false){}
};
typedef struct mVoxelEdge mVoxel_edge;

struct mVoxel{
    std::array<mVoxel_vertex *,8> vertex_ptrs;  // each voxel has 8 vertices
    std::array<mVoxel_edge *, 12> voxel_edges_ptrs;
    unsigned int bitcode;
};
typedef struct mVoxel mVoxel;







class marching_cube
{
public:
    const float unit_voxel_length;   // resolution
    float total_z_length = 3.0f;
    float total_x_length = 3.0f;
    float total_y_length = 3.0f;

    // voxels number along 3 axis;
    size_t voxelx_n;
    size_t voxely_n;
    size_t voxelz_n;

    Vector3f origin = Vector3f(0.0f,0.0f,-1.0f);  // this origin is the buttom center of the
                                                         //cuboid we want to reconstruct

    marching_cube(float unit_length);
    virtual ~marching_cube();

    void start_marching_cube();
    void output_marching_vertices(std::vector<float>& output_vertices);
    void output_marching_indices(std::vector<unsigned int>& output_vertices);
    void output_marching_vertices_and_normals(std::vector<float>& output_vertices_and_normals);



protected:


    // vertices number along 3 axis;
    size_t voxel_verticesx_n;
    size_t voxel_verticesy_n;
    size_t voxel_verticesz_n;



    // private method, which only used by internal.
    virtual void compute_vertex_normal(const Vector3f& vertex, Vector3f& normal) = 0;
    virtual void compute_vertices_phi() = 0;
    void initialize_vertices();
    void initialize_voxels();
    void initialize_edges();
    void mark_vertices();
    void generate_bitcode();
    virtual void bitcode_to_mesh_vertices();
    virtual void insect_vertex_to_edges( mVoxel_edge& edge);
    void linear_interpolate_vertex_pos(const mVoxel_vertex &vertex1, const mVoxel_vertex &vertex2, Vector3f& result_pos);


    std::vector<mVoxel_vertex> voxel_vertices;
    std::list<mVoxel> voxels;
    std::vector<mVoxel_edge> edges_vector_x;
    std::vector<mVoxel_edge> edges_vector_y;
    std::vector<mVoxel_edge> edges_vector_z;



    // store internel triangle face and mesh vertex
    std::list<mMesh_vertex> mesh_vertex_vector;
    std::list<mMesh_triangle> mesh_triangle_vector;
};

#endif // MARCHING_CUBE_H
