#include "marching_cube.hpp"
#include "marching_cubes_lut.hpp"
#include <bitset>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <vector>

using namespace std;

//-------------------------------- virtual functions-------------------------------------
// should define according to the data
/*
void marching_cube::compute_vertex_normal(const Vector3f& vertex, Vector3f& normal)
{// this is for the sphere, which center is the origin points
    Vector3f ori={0.0f,0.0f,0.0f};
    normal = (vertex - ori).normalized();

    // debug
}

// this function now only used to compute the unit sphere.
void marching_cube::compute_vertices_phi(){
    size_t len = voxel_vertices.size();
    for(size_t i=0; i< len; ++i)
    {
        float dist = voxel_vertices[i].position.norm();
        voxel_vertices[i].phi = 1 - dist;
    }
    return;
}
*/
//--------------------------------- virtual functions end-------------------------------


marching_cube::marching_cube(float unit_length):unit_voxel_length(unit_length)
{
    voxelx_n = static_cast<size_t>(total_x_length/unit_voxel_length);
    voxely_n = static_cast<size_t>(total_y_length/unit_voxel_length);
    voxelz_n = static_cast<size_t>(total_z_length/unit_voxel_length);

    voxel_verticesx_n = voxelx_n+1;
    voxel_verticesy_n = voxely_n+1;
    voxel_verticesz_n = voxelz_n+1;
    voxel_vertices.reserve(voxel_verticesx_n*voxel_verticesy_n*voxel_verticesz_n);
    edges_vector_x.reserve(voxelx_n*(voxely_n+1)*(voxelz_n+1));
    edges_vector_y.reserve((voxelx_n+1)*voxely_n*(voxelz_n+1));
    edges_vector_z.reserve((voxelx_n+1)*(voxely_n+1)*voxelz_n);


    cout<<"size of mvoxel_vertex "<<sizeof(mVoxel_vertex)<<endl;
    cout<<"size of mvoxel "<<sizeof(mVoxel)<<endl;
    cout<<"size of mMesh_vertex "<<sizeof(mMesh_vertex)<<endl;
    cout<<"size of mMesh_triangle "<<sizeof(mMesh_triangle)<<endl;
}

marching_cube::~marching_cube()
{
	voxel_vertices.shrink_to_fit();
	voxels.clear();

	mesh_vertex_vector.clear();
	mesh_triangle_vector.clear();
}

void marching_cube::start_marching_cube(){
    this->initialize_vertices();
    this->compute_vertices_phi(); // for each vertices, compute its phi value
    this->mark_vertices();
    this->initialize_edges();
    this->initialize_voxels();
    this->generate_bitcode();
    //
    this->bitcode_to_mesh_vertices();

    cout<<"total memeory for mvoxel_vertex is "<<sizeof(mVoxel_vertex)*voxel_vertices.size()<<endl;
    cout<<"total memeory for mvoxel_edge is "<<sizeof(mVoxel_edge)*(  edges_vector_x.size()
                                                                    + edges_vector_y.size()
                                                                    + edges_vector_z.size()) << endl;

    cout<<"total memeory for mvoxel is "<<sizeof(mVoxel)*voxels.size()<<endl;
    cout<<"total memeory for mesh_vertex is "<<sizeof(mMesh_vertex)*mesh_vertex_vector.size()<<endl;
    cout<<"total memeory for mesh_triangle is "<<sizeof(mMesh_triangle)*mesh_triangle_vector.size()<<endl;

    //release mvoxel_vertex space
    //std::vector<mVoxel_vertex>().swap(voxel_vertices);
}

void marching_cube::output_marching_vertices(std::vector<float>& output_vertices){
    // now assign mesh_vertex_vector to output vertex

    // wtf ...
    //if (output_vertices.empty())
    //	output_vertices.reserve(len*3);

    if (!output_vertices.empty())
        output_vertices.clear();

    for(auto it = mesh_vertex_vector.begin(); it!=mesh_vertex_vector.end();++it)
    {
        output_vertices.push_back((*it).position[0]);
        output_vertices.push_back((*it).position[1]);
        output_vertices.push_back((*it).position[2]);
    }
    return;
}

void marching_cube::output_marching_vertices_and_normals(std::vector<float>& output_vertices_and_normals){
    // now assign mesh vertex and its normal to output


    //if (output_vertices_and_normals.empty())
    // 	output_vertices_and_normals.reserve(len*6);

    if (!output_vertices_and_normals.empty())
    	output_vertices_and_normals.clear();

    for(auto it = mesh_vertex_vector.begin(); it!=mesh_vertex_vector.end();++it)
    {
        output_vertices_and_normals.push_back((*it).position[0]);
        output_vertices_and_normals.push_back((*it).position[1]);
        output_vertices_and_normals.push_back((*it).position[2]);

        output_vertices_and_normals.push_back(it->normal[0]);
        output_vertices_and_normals.push_back(it->normal[1]);
        output_vertices_and_normals.push_back(it->normal[2]);
    }
    return;
}


void marching_cube::output_marching_indices(std::vector<unsigned int>& output_indices){
    // now assign mesh_triangle_vector to output triangle

    //if (output_indices.empty())
    //	output_indices.reserve(len*3);

    if (!output_indices.empty())
        output_indices.clear();

    for(auto it = mesh_triangle_vector.begin();it!=mesh_triangle_vector.end();++it)
    {
        output_indices.push_back(it->vertex_ids[0]);
        output_indices.push_back(it->vertex_ids[1]);
        output_indices.push_back(it->vertex_ids[2]);
    }
    return;
}

void marching_cube::linear_interpolate_vertex_pos(const mVoxel_vertex& vertex1,
                                                  const mVoxel_vertex& vertex2,
                                                  Vector3f& result_pos){
    result_pos = (-vertex1.phi/(vertex2.phi - vertex1.phi))*(vertex2.position - vertex1.position)+vertex1.position;
}

void marching_cube::insect_vertex_to_edges( mVoxel_edge& edge){
    if(edge.has_mesh_vertex)  // if this edge already has a vertex, do nothing but return directly
        return;
    else{
        unsigned int len = mesh_vertex_vector.size();
        unsigned int vertex_id = len;

        // compute its postion, now we just use midpoint
        //Vector3f vertex_pos = (edge.vertex1_ptr->position + edge.vertex2_ptr->position)/2.0f;

        // use linear interpolate
        Vector3f vertex_pos;
        linear_interpolate_vertex_pos(*(edge.vertex1_ptr), *(edge.vertex2_ptr),vertex_pos);

        Vector3f vertex_normal;
        compute_vertex_normal(vertex_pos,vertex_normal);

        mMesh_vertex mv(vertex_pos,vertex_normal,vertex_id);
        mesh_vertex_vector.push_back(mv);
        edge.meshVertexptr = &(mesh_vertex_vector.back()); // note in vector index is from 0.
        edge.has_mesh_vertex = true;
    }
    return;
}

void marching_cube::bitcode_to_mesh_vertices(){

	if (!mesh_vertex_vector.empty())
		mesh_vertex_vector.clear();

	if (!mesh_triangle_vector.empty())
		mesh_triangle_vector.clear();

    for(auto it = voxels.begin(); it!=voxels.end();++it)
    {
        const char* cur_voxel_lut = marching_cubes_lut[it->bitcode];

        for(size_t j = 0; j < 16; j=j+3) //because in the marching_cubes_lut is 256*16
                                            // j = j+3, because 3 entries build a triangle
        {
            if(cur_voxel_lut[j]==-1)
                break; // if we see the -1, means we already read all triangle in this voxel
            else
            {
                 //mVoxel& vv = *it;
                mVoxel_edge& edge1 = *(it->voxel_edges_ptrs[cur_voxel_lut[j]]);
                mVoxel_edge& edge2 = *(it->voxel_edges_ptrs[cur_voxel_lut[j+1]]);
                mVoxel_edge& edge3 = *(it->voxel_edges_ptrs[cur_voxel_lut[j+2]]);

               // compute postion for insect vertiecs, accroding 3 edges
                insect_vertex_to_edges(edge1);
                insect_vertex_to_edges(edge2);
                insect_vertex_to_edges(edge3);


                // form triangle by using these 3 vertex in these 3 edges;
                mMesh_vertex* v1 = edge1.meshVertexptr;
                mMesh_vertex* v2 = edge2.meshVertexptr;
                mMesh_vertex* v3 = edge3.meshVertexptr;

                 // about the count-clock wise ??  I think the count-clock wise is according to the normal direction..
                 mMesh_triangle mt;
                 mt.vertex_ids[0] = v1->id;
                 mt.vertex_ids[1] = v2->id;
                 mt.vertex_ids[2] = v3->id;

                 // if normal is on the opposite direction with the cross product of vertices(that is, dot product < 0)
                 Vector3f e1 = v2->position - v1->position ;
                 Vector3f e2 = v3->position - v1->position ;

                 if (v1->normal.dot(e1.cross(e2)) < 0.0)
                 {
                	 unsigned int tmp = mt.vertex_ids[0];
                	 mt.vertex_ids[0] = mt.vertex_ids[1];
                	 mt.vertex_ids[1] = tmp;
                 }

                 mesh_triangle_vector.push_back(mt);
            }
        }
    }
}



void marching_cube::mark_vertices(){
    size_t len = voxel_vertices.size();
    for(size_t i=0;i<len;++i )
    {
        if(voxel_vertices[i].phi > 0.0f)
            voxel_vertices[i].is_inside = true;
        else
            voxel_vertices[i].is_inside = false;
    }
}

void marching_cube::generate_bitcode(){
    for(auto it = voxels.begin(); it!=voxels.end();++it)
    {
        mVoxel& vox = *it;
        bitset<8> bitcode(0x00); // initialize to all 0
        for(size_t j = 0; j<8;++j) // since for each voxel we have 8 vertices
        {
            mVoxel_vertex* vertex_ptr = vox.vertex_ptrs[j];
            if(vertex_ptr->is_inside)
                bitcode[j] = 1;
        }
        vox.bitcode =static_cast<unsigned int>(bitcode.to_ulong());
    }
    return;
}

// here we initialize each voxels, assign vertices to voxel
void marching_cube::initialize_voxels()
{
	if (!voxels.empty())
		voxels.clear();

    for(size_t z = 0; z<voxelz_n;++z)
    {
        for(size_t y = 0; y<voxely_n;++y)
        {
            for(size_t x = 0; x<voxelx_n ;++x)
            {
                mVoxel v;
                bool is_useful_voxel = false;

                size_t voxel_front_left_bottom_index = x+y*voxel_verticesx_n+z*voxel_verticesx_n*voxel_verticesy_n;
                v.vertex_ptrs[0]=&(voxel_vertices[voxel_front_left_bottom_index]);
                v.vertex_ptrs[1]=&(voxel_vertices[voxel_front_left_bottom_index+1]);
                v.vertex_ptrs[2]=&(voxel_vertices[voxel_front_left_bottom_index+voxel_verticesx_n+1]);
                v.vertex_ptrs[3]=&(voxel_vertices[voxel_front_left_bottom_index+voxel_verticesx_n]);

                size_t voxel_front_left_top_index = voxel_front_left_bottom_index+voxel_verticesy_n*voxel_verticesx_n;
                v.vertex_ptrs[4]=&(voxel_vertices[voxel_front_left_top_index]);
                v.vertex_ptrs[5]=&(voxel_vertices[voxel_front_left_top_index+1]);
                v.vertex_ptrs[6]=&(voxel_vertices[voxel_front_left_top_index+voxel_verticesx_n+1]);
                v.vertex_ptrs[7]=&(voxel_vertices[voxel_front_left_top_index+voxel_verticesx_n]);

                // determine if it is a useful voxel
                if(v.vertex_ptrs[0]->is_inside)
                {// if the first vertex is inside, then we try to find if other vertiecs are outside
                    for(int i=1;i<8;++i)
                    {
                        if(!v.vertex_ptrs[i]->is_inside)
                        {// if outside, it is a useful voxel
                            is_useful_voxel = true;
                            break;
                        }
                        else
                        {
                            is_useful_voxel = false;
                        }
                    }
                }
                else
                {
                    for(int i=1;i<8;++i)
                    {
                        if(v.vertex_ptrs[i]->is_inside)
                        {
                            is_useful_voxel = true;
                            break;
                        }
                        else
                        {
                            is_useful_voxel = false;
                        }
                    }
                }

                if(is_useful_voxel)
                {//only store useful voxel
                    size_t voxel_index_x = x+y*voxelx_n+z*voxelx_n*voxel_verticesy_n;
                    v.voxel_edges_ptrs[0] = &edges_vector_x[voxel_index_x];
                    v.voxel_edges_ptrs[2] = &edges_vector_x[voxel_index_x + voxelx_n];
                    v.voxel_edges_ptrs[4] = &edges_vector_x[voxel_index_x + voxelx_n*(voxely_n+1)];
                    v.voxel_edges_ptrs[6] = &edges_vector_x[voxel_index_x + voxelx_n*(voxely_n+1) + voxelx_n];

                    size_t voxel_index_y = x+y*voxel_verticesx_n+z*voxely_n*voxel_verticesx_n;
                    v.voxel_edges_ptrs[3] = &edges_vector_y[voxel_index_y];
                    v.voxel_edges_ptrs[1] = &edges_vector_y[voxel_index_y+1];
                    v.voxel_edges_ptrs[5] = &edges_vector_y[voxel_index_y+1 + (voxelx_n+1)*voxely_n ];
                    v.voxel_edges_ptrs[7] = &edges_vector_y[voxel_index_y+ (voxelx_n+1)*voxely_n ];

                    size_t voxel_index_z = x+y*voxel_verticesx_n+z*voxel_verticesy_n*voxel_verticesx_n;
                    v.voxel_edges_ptrs[8] = &edges_vector_z[voxel_index_z];
                    v.voxel_edges_ptrs[9] = &edges_vector_z[voxel_index_z+1];
                    v.voxel_edges_ptrs[10] = &edges_vector_z[voxel_index_z+1+voxelx_n+1];
                    v.voxel_edges_ptrs[11] = &edges_vector_z[voxel_index_z+voxelx_n+1];
                    voxels.push_back(v);
                }
            }
        }
    }
    return;
}


void marching_cube::initialize_vertices(){
	if (!voxel_vertices.empty())
	{
		voxel_vertices.clear();
	}

    voxel_vertices.reserve(voxel_verticesz_n * voxel_verticesy_n * voxel_verticesx_n);    

    float step_size = unit_voxel_length;
    float x_half_extent = step_size*voxelx_n/2.0;
    float y_half_extent = step_size*voxely_n/2.0;
    //
    for (size_t k=0; k<voxel_verticesz_n; ++k)
    {
        for (size_t j=0; j<voxel_verticesy_n; ++j)
        {
            for (size_t i=0; i<voxel_verticesx_n; ++i)
            {
                mVoxel_vertex p;

                float x = (i)*step_size-x_half_extent + origin[0];
                float y = (j)*step_size-y_half_extent + origin[1];
                float z = (k)*step_size + origin[2];

                p.position = Vector3f(x,y,z);
                p.phi = -1;
                p.is_inside = false;
                voxel_vertices.push_back(p);
            }
        }
    }
    return;
}


void marching_cube::initialize_edges(){
    if(voxel_vertices.empty())
    {
        cout<<"Oppps! voxel_vertices is empty, you should at first initialize_vertices"<<endl;
        return;
    }
    // first initialize all edge along x axis
    if(!edges_vector_x.empty())
    {
        edges_vector_x.clear();
        edges_vector_x.reserve(voxelx_n*(voxely_n+1)*(voxelz_n+1));
    }

    for (size_t k=0; k<voxelz_n+1; ++k) // note, for edges paralel with x axis, its number at y and z is the same as vertice
    {
        for (size_t j=0; j<voxely_n+1; ++j)
        {
            for (size_t i=0; i<voxelx_n; ++i)
            {
                mVoxel_edge edge;
                edge.has_mesh_vertex = false;
                size_t vertex_index = i + j*voxel_verticesx_n + k*voxel_verticesx_n*voxel_verticesy_n;
                edge.vertex1_ptr = &voxel_vertices[vertex_index];
                edge.vertex2_ptr = &voxel_vertices[vertex_index+1];
                edges_vector_x.push_back(edge);
            }
        }
    }

    // initialized all edge along y axis
    if(!edges_vector_y.empty())
    {
        edges_vector_y.clear();
        edges_vector_y.reserve((voxelx_n+1)*voxely_n*(voxelz_n+1));
    }
    for (size_t k=0; k<voxelz_n+1; ++k)
    {
        for (size_t j=0; j<voxely_n; ++j)
        {
            for (size_t i=0; i<voxelx_n+1; ++i)
            {
                mVoxel_edge edge;
                edge.has_mesh_vertex = false;
                size_t vertex_index = i + j*voxel_verticesx_n + k*voxel_verticesx_n*voxel_verticesy_n;
                edge.vertex1_ptr = &voxel_vertices[vertex_index];
                edge.vertex2_ptr = &voxel_vertices[vertex_index+1*voxel_verticesx_n];
                edges_vector_y.push_back(edge);
            }
        }
    }

    // initialized all edge along z axis
    if(!edges_vector_z.empty())
    {
        edges_vector_z.clear();
        edges_vector_z.reserve((voxelx_n+1)*(voxely_n+1)*voxelz_n);
    }
    for (size_t k=0; k<voxelz_n; ++k)
    {
        for (size_t j=0; j<voxely_n+1; ++j)
        {
            for (size_t i=0; i<voxelx_n+1; ++i)
            {
                mVoxel_edge edge;
                edge.has_mesh_vertex = false;
                size_t vertex_index = i + j*voxel_verticesx_n + k*voxel_verticesx_n*voxel_verticesy_n;
                edge.vertex1_ptr = &voxel_vertices[vertex_index];
                edge.vertex2_ptr = &voxel_vertices[vertex_index+voxel_verticesx_n*voxel_verticesy_n];
                edges_vector_z.push_back(edge);
            }
        }
    }


}

