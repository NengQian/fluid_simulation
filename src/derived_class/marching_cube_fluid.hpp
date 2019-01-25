#ifndef MARCHING_CUBE_FLUID
#define MARCHING_CUBE_FLUID

#include "marching_cube.hpp"
#include "sim_record.hpp"
#include "Particle.hpp"
#include "visual.hpp"
#include "math_types.hpp"
#include "NeighborSearcher.hpp"
#include "KernelHandler.hpp"
#include "ParticleFunc.hpp"
#include "marching_cubes_lut.hpp"

#include <math.h>       /* sqrt */
#include <iostream>

#define MAX 2147483647
#define MIN -2147483648

using namespace std;

class marching_cube_fluid: public marching_cube
{
public:
    bool end = false;
    int count = 0;
    Real c;
	int total_lost = 0;

	marching_cube_fluid(float unit_length, Real c, std::string input_file) : marching_cube(unit_length), c(c), pf(1000, 1000, 0.08)
	{
		load_particle_series(input_file);
		load_next_particles();
		min_x = min_y = min_z = MAX;
		max_x = max_y = max_z = MIN;
		set_grid_size();

		ns.set_neighbor_search_radius(search_radius);
		kh.set_neighbor_search_radius(search_radius);
    }

	~marching_cube_fluid() noexcept
	{
		voxel_vertices.shrink_to_fit();
		voxels.clear();

		mesh_vertex_vector.clear();
		mesh_triangle_vector.clear();

	    particles_series.shrink_to_fit();
	    current_particles.shrink_to_fit();
	    grid_position.shrink_to_fit();
	    particle_positions.shrink_to_fit();
	}

	virtual void insect_vertex_to_edges( mVoxel_edge& edge) override
	{
	    if (edge.has_mesh_vertex)  // if this edge already has a vertex, do nothing but return directly
	        return;
	    else{
	        unsigned int len = mesh_vertex_vector.size();
	        unsigned int vertex_id = len;

	        // compute its postion, now we just use midpoint
	        //Vector3f vertex_pos = (edge.vertex1_ptr->position + edge.vertex2_ptr->position)/2.0f;

	        // use linear interpolate
	        Vector3f vertex_pos;
	        linear_interpolate_vertex_pos(*(edge.vertex1_ptr), *(edge.vertex2_ptr), vertex_pos);

	        Vector3f vertex_normal(0.0f, 0.0f, 0.0f);

	        mMesh_vertex mv(vertex_pos,vertex_normal,vertex_id);
	        mesh_vertex_vector.push_back(mv);
	        edge.meshVertexptr = &(mesh_vertex_vector.back()); // note in vector index is from 0.
	        edge.has_mesh_vertex = true;
	    }
	    return;
	}

	virtual void bitcode_to_mesh_vertices() override
	{
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

	                 mVoxel_edge& edge1 = it->voxel_edges[cur_voxel_lut[j]];
	                 mVoxel_edge& edge2 = it->voxel_edges[cur_voxel_lut[j+1]];
	                 mVoxel_edge& edge3 = it->voxel_edges[cur_voxel_lut[j+2]];

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

	                 mesh_triangle_vector.push_back(mt);
	            }
	        }
	    }

	    // prepare data for further neighbor search (and further for normal computation)
	    std::vector<RealVector3> mesh_vertex_pos;
	    for(auto it = mesh_vertex_vector.begin(); it != mesh_vertex_vector.end(); ++it)
	    {
	    	mesh_vertex_pos.push_back(RealVector3(it->position[0], it->position[1], it->position[2]));
	    }

    	update_particle_positions();
	    std::vector<std::vector<size_t>> neighbor_indices = ns.find_neighbors_within_radius(mesh_vertex_pos);

	    compute_vertex_normal(mesh_vertex_pos, neighbor_indices);

	    for(auto it = mesh_triangle_vector.begin(); it != mesh_triangle_vector.end(); ++it)
	    {
	    	auto it_vtx = mesh_vertex_vector.begin();
	        auto it1 = it_vtx;
	        std::advance(it1, it->vertex_ids[0]);

	        auto it2 = it_vtx;
	        std::advance(it2, it->vertex_ids[1]);

	        auto it3 = it_vtx;
	        std::advance(it3, it->vertex_ids[2]);

	    	Vector3f e1 = it2->position - it1->position ;
	    	Vector3f e2 = it3->position - it1->position ;

	    	if (it1->normal.dot(e1.cross(e2)) < 0.0)
	    	{
	    		unsigned int tmp = it->vertex_ids[0];
	    		it->vertex_ids[0] = it->vertex_ids[1];
	    		it->vertex_ids[1] = tmp;
	    	}
	    }
	}

	void compute_vertex_normal(std::vector<RealVector3>& mesh_vertex_pos, std::vector<std::vector<size_t>>& neighbor_indices)
    {
		size_t i = 0;
	    for(auto it = mesh_vertex_vector.begin(); it != mesh_vertex_vector.end(); ++it)
	    {
	    	for (size_t j=0; j<neighbor_indices[i].size(); ++j)
	    	{
	    		size_t idx = neighbor_indices[i][j];

	    		RealVector3 gd = kh.gradient_of_kernel(mesh_vertex_pos[i], particle_positions[idx]);
	    		Vector3f gf(static_cast<float>(gd[0]), static_cast<float>(gd[1]), static_cast<float>(gd[2]));
	    		it->normal -= gf;
	    	}
	    	it->normal.normalize();

	    	++i;
	    }
	}

    virtual void compute_vertex_normal(const Vector3f& vertex, Vector3f& normal) override
    {

    }

    virtual void compute_vertices_phi() override
    {
    	update_particle_positions();
		std::vector<std::vector<size_t>> neighbor_indices = ns.find_neighbors_within_radius(grid_position);

		// recompute density, ignore boundary particles
		std::vector< std::vector<size_t> > tmp_n = ns.find_neighbors_within_radius( true );

		pf.update_density(tmp_n, current_particles, search_radius);

		/*
		for (size_t i=0; i<neighbor_indices.size(); ++i)
		{
			std::cout << "neighbor of " << i << std::endl;
			for (size_t j=0; j<neighbor_indices[i].size(); ++j)
				std::cout << j << " << ";

			std::cout << std::endl;
		}
		*/

		//std::cout << "22222" << std::endl;


        size_t len = voxel_vertices.size();
        for (size_t i=0; i<len; ++i)
        {
        	//std::cout << "i = " << i << std::endl;
        	RealVector3 grid_vertex = grid_position[i];

    		voxel_vertices[i].phi = -c;

    		//std::cout << "33333" << std::endl;

        	for (size_t j=0; j<neighbor_indices[i].size(); ++j)
        	{
            	//std::cout << "j = " << j << std::endl;

        		//std::cout << "44444" << std::endl;

        		int idx = neighbor_indices[i][j];

        		mParticle p = current_particles[idx];

        		voxel_vertices[i].phi += p.mass / p.density * kh.compute_kernel(grid_vertex, p.position);

        		//std::cout << "55555" << std::endl;

        	}

        }

        return;
    }

    void start_marching_cube()
    {
		pick_up_particles();
        initialize_vertices();

        save_grid_position();
		
        compute_vertices_phi(); // for each vertices, compute its phi value
        mark_vertices();
        initialize_voxels();
        generate_bitcode();

        bitcode_to_mesh_vertices();
    }

    void update_marching_cube()
    {
    	load_next_particles();
    	if (end)
    		return;

		update_grid_size();
		pick_up_particles();
        initialize_vertices();

        save_grid_position();

    	compute_vertices_phi(); // for each vertices, compute its phi value
    	mark_vertices();
    	initialize_voxels();
    	generate_bitcode();
        //
    	bitcode_to_mesh_vertices();
    }

	void pick_up_particles()
	{
		total_lost = 0;

		auto dps = discarded_particles_series[count];

		if (!dps.empty())
		{
			for (auto& dp : dps)
			{
				if (static_cast<float>(dp.position[0])-origin[0] < total_x_length*0.5f && static_cast<float>(dp.position[1])-origin[1] < total_y_length*0.5f && static_cast<float>(dp.position[2])-origin[2] < total_z_length)
				{
					// pick it up again
					current_particles.push_back(dp);
				} else {
					++total_lost;
				}
			}
		}
	}

	void count_exclusive_particles_in_fixed_box()
	{
		total_lost = 0;

		auto dps = discarded_particles_series[count];

		if (!dps.empty())
		{
			for (auto& dp : dps)
			{
				if (static_cast<float>(dp.position[0])-origin[0] < total_x_length*0.5f && static_cast<float>(dp.position[1])-origin[1] < total_y_length*0.5f && static_cast<float>(dp.position[2])-origin[2] < total_z_length)
				{
					;
				} else {
					++total_lost;
				}
			}
		}

		auto ps = particles_series[count];

		if (!ps.empty())
		{
			for (auto& p : ps)
			{
				if (static_cast<float>(p.position[0])-origin[0] < total_x_length*0.5f && static_cast<float>(p.position[1])-origin[1] < total_y_length*0.5f && static_cast<float>(p.position[2])-origin[2] < total_z_length)
				{
					;
				} else {
					++total_lost;
				}
			}
		}		
	}

	void load_next_particles()
    {
    	if (count >= particles_series.size())
    	{
    		end = true;
        	//std::cout << "end" << std::endl;
        	return;
    	}

    	current_particles = particles_series[count];
    	//std::cout << "count = " << count << std::endl;
    }

    void update_grid_size()
    {
    	Real cmin_x, cmin_y, cmin_z, cmax_x, cmax_y, cmax_z;
    	cmin_x = cmin_y = cmin_z = MAX;
    	cmax_x = cmax_y = cmax_z = MIN;

		for (auto& p : current_particles)
		{
			if (p.position[0] > cmax_x)
				cmax_x = p.position[0];
			if (p.position[0] < cmin_x)
				cmin_x = p.position[0];

			if (p.position[1] > cmax_y)
				cmax_y = p.position[1];
			if (p.position[1] < cmin_y)
				cmin_y = p.position[1];

			if (p.position[2] > cmax_z)
				cmax_z = p.position[2];
			if (p.position[2] < cmin_z)
				cmin_z = p.position[2];
		}

		double du = static_cast<double>(unit_voxel_length);

		double max_x_unit = ceil((cmax_x-max_x)/du*0.5) + 1;
		double min_x_unit = floor((cmin_x-min_x)/du*0.5) - 1;

		double max_y_unit = ceil((cmax_y-max_y)/du*0.5) + 1;
		double min_y_unit = floor((cmin_y-min_y)/du*0.5) - 1;

		double max_z_unit = ceil((cmax_z-max_z)/du*0.5) + 1;
		double min_z_unit = floor((cmin_z-min_z)/du*0.5) - 1;

		// update min, max
		min_x = min_x + min_x_unit * 2.0 * du;
		min_y = min_y + min_y_unit * 2.0 * du;
		min_z = min_z + min_z_unit * 2.0 * du;

		max_x = max_x + max_x_unit * 2.0 * du;
		max_y = max_y + max_y_unit * 2.0 * du;
		max_z = max_z + max_z_unit * 2.0 * du;

    	total_x_length = static_cast<float>(max_x - min_x);
    	total_y_length = static_cast<float>(max_y - min_y);
		total_z_length = static_cast<float>(max_z - min_z);

		origin = Vector3f(static_cast<float>(max_x + min_x)*0.5f, static_cast<float>(max_y + min_y)*0.5f, static_cast<float>(min_z));

		update_voxel();

		/*
		std::cout << "in update" << std::endl;

		std::cout << "total_x_length = " << total_x_length << std::endl;
		std::cout << "total_y_length = " << total_y_length << std::endl;
		std::cout << "total_z_length = " << total_z_length << std::endl;

		std::cout << "ori = (" << origin[0] << "," << origin[1] << "," << origin[2] << ")" << std::endl;
		*/
    }

    void set_grid_size()
    {
		for (auto& p : current_particles)
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

    	total_x_length = static_cast<float>(max_x_unit-min_x_unit);
    	total_y_length = static_cast<float>(max_y_unit-min_y_unit);
		total_z_length = static_cast<float>(max_z_unit-min_z_unit);

		origin = Vector3f(static_cast<float>(max_x_unit+min_x_unit)*0.5f, static_cast<float>(max_y_unit+min_y_unit)*0.5f, static_cast<float>(min_z_unit));

		update_voxel();
		/*
		std::cout << "in set" << std::endl;

		std::cout << "total_x_length = " << total_x_length << std::endl;
		std::cout << "total_y_length = " << total_y_length << std::endl;
		std::cout << "total_z_length = " << total_z_length << std::endl;

		std::cout << "ori = (" << origin[0] << "," << origin[1] << "," << origin[2] << ")" << std::endl;
		*/
    }

    void update_voxel()
    {
	    voxelx_n = static_cast<size_t>(ceil(total_x_length/unit_voxel_length));
	    voxely_n = static_cast<size_t>(ceil(total_y_length/unit_voxel_length));
	    voxelz_n = static_cast<size_t>(ceil(total_z_length/unit_voxel_length));

	    voxel_verticesx_n = voxelx_n+1;
	    voxel_verticesy_n = voxely_n+1;
	    voxel_verticesz_n = voxelz_n+1;
	    //voxel_vertices.reserve(voxel_verticesx_n*voxel_verticesy_n*voxel_verticesz_n);
    }

protected:
	NeighborSearcher ns;
	KernelHandler kh;
	ParticleFunc pf;

    std::vector<std::vector<mParticle>> particles_series;
	std::vector<std::vector<mParticle>> discarded_particles_series;

    std::vector<mParticle> current_particles;
    std::vector<RealVector3> grid_position;
    std::vector<RealVector3> particle_positions;

    Real search_radius;
    Real particle_unit;

    Real min_x, max_x, min_y, max_y, min_z, max_z;

    void save_grid_position()
    {
    	//std::cout << "in grid setting" << std::endl;

    	if (!grid_position.empty())
    		grid_position.clear();

    	for (unsigned int i=0; i<voxel_vertices.size(); ++i)
    	{
    		grid_position.push_back(RealVector3(static_cast<Real>(voxel_vertices[i].position[0]), static_cast<Real>(voxel_vertices[i].position[1]), static_cast<Real>(voxel_vertices[i].position[2])));
    	}
    }

    void update_particle_positions()
    {
    	if (!particle_positions.empty())
    		particle_positions.clear();

    	if (particle_positions.empty())
    	{
        	for (int i=0; i<current_particles.size(); ++i)
        	{
        		particle_positions.push_back(current_particles[i].position);
        	}
    	}

    	ns.set_particles_ptr(particle_positions);
    }

    void load_particle_series(std::string fs)
    {
    	Visualization simData(fs);

    	particles_series.clear();
		discarded_particles_series.clear();

    	// set neighbor search radius
    	search_radius = simData.sim_rec.unit_particle_length * simData.eta * 2;
    	particle_unit = simData.sim_rec.unit_particle_length;

    	for (int i=0; i<simData.total_frame_num; ++i)
    	{
    		//std::vector<mParticle> ps = simData.sim_rec.states[i].particles;

			std::vector<mParticle> ps;
			std::vector<mParticle> dps;
			for (auto& p : simData.sim_rec.states[i].particles)
			{
				if (p.density >= 185.0)
					ps.push_back(p);
				else {
					dps.push_back(p);
				}
			}

    		particles_series.push_back(ps);
			discarded_particles_series.push_back(dps);
			//std::cout << "vector size: " << discarded_particles_series.size() << std::endl;
    	}
    }
};

#endif // MARCHING_CUBE_FLUID

