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

#include <math.h>       /* sqrt */
#include <iostream>


using namespace std;

class marching_cube_fluid: public marching_cube
{
public:
    bool end = false;
    int count = 0;

	marching_cube_fluid(float unit_length, Real c, std::string input_file) : marching_cube(unit_length), c(c), pf(1000, 1000, 0.08)
	{
		load_particle_series(input_file);
		set_grid_size();

		ns.set_neighbor_search_radius(search_radius);
		kh.set_neighbor_search_radius(search_radius);

		load_next_particles();
    }

	~marching_cube_fluid()
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
    	compute_vertices_phi(); // for each vertices, compute its phi value
    	mark_vertices();
    	initialize_voxels();
    	generate_bitcode();
        //
    	bitcode_to_mesh_vertices();
    }

protected:
	NeighborSearcher ns;
	KernelHandler kh;
	ParticleFunc pf;

    std::vector<std::vector<mParticle>> particles_series;
    std::vector<mParticle> current_particles;
    std::vector<RealVector3> grid_position;
    std::vector<RealVector3> particle_positions;

    Real c;
    Real search_radius;

    void save_grid_position()
    {
    	std::cout << "in grid setting" << std::endl;

    	if (!grid_position.empty())
    		grid_position.clear();

    	for (unsigned int i=0; i<voxel_vertices.size(); ++i)
    	{
    		grid_position.push_back(RealVector3(static_cast<Real>(voxel_vertices[i].position[0]), static_cast<Real>(voxel_vertices[i].position[1]), static_cast<Real>(voxel_vertices[i].position[2])));
    	}
    }

    void load_next_particles()
    {
    	current_particles = particles_series[count];

    	if (count < particles_series.size()-1)
    		++count;
    	else
    		end = true;

    	std::cout << "count = " << count << std::endl;
    }

    void update_particle_positions()
    {
    	if (particle_positions.empty())
    	{
        	for (int i=0; i<current_particles.size(); ++i)
        	{
        		particle_positions.push_back(current_particles[i].position);
        	}
    	}
    	else {
        	for (int i=0; i<particle_positions.size(); ++i)
        	{
        		/*
        		std::cout << "before: ";
        		std::cout << particle_positions[i][0] << ", ";
        		std::cout << particle_positions[i][1] << ", ";
        		std::cout << particle_positions[i][2] << std::endl;
        		*/

        		particle_positions[i][0] = current_particles[i].position[0];
        		particle_positions[i][1] = current_particles[i].position[1];
        		particle_positions[i][2] = current_particles[i].position[2];

        		/*
        		std::cout << "after: ";
        		std::cout << particle_positions[i][0] << ", ";
        		std::cout << particle_positions[i][1] << ", ";
        		std::cout << particle_positions[i][2] << std::endl;
        		*/
        	}
    	}

    	ns.set_particles_ptr(particle_positions);
    }

    void load_particle_series(std::string fs)
    {
    	Visualization simData(fs);

    	particles_series.clear();

    	// set neighbor search radius
    	search_radius = simData.sim_rec.unit_particle_length * 2.4;

    	for (int i=0; i<simData.total_frame_num; ++i)
    	{
    		std::vector<mParticle> ps = simData.sim_rec.states[i].particles;

    		particles_series.push_back(ps);
    	}
    }

    void set_grid_size()
    {
    	Real min_x, min_y, min_z, max_x, max_y, max_z;
    	min_x = min_y = min_z = 100000000;
    	max_x = max_y = max_z = -100000000;

    	for (auto& ps : particles_series)
    	{
    		for (auto& p : ps)
    		{
    			if (p.position[0] > max_x)
    				max_x = p.position[0];
    			else if (p.position[0] < min_x)
    				min_x = p.position[0];

    			if (p.position[1] > max_y)
    				max_y = p.position[1];
    			else if (p.position[1] < min_y)
    				min_y = p.position[1];

    			if (p.position[2] > max_z)
    				max_z = p.position[2];
    			else if (p.position[2] < min_z)
    				min_z = p.position[2];
    		}
    	}

    	total_x_length = static_cast<int>(max_x - min_x) + 1.0f;
		total_y_length = static_cast<int>(max_y - min_y) + 1.0f;
		total_z_length = static_cast<int>(max_z - min_z) + 1.0f;

		origin = Vector3f(static_cast<float>((max_x + min_x)/2.0), static_cast<float>((max_y + min_y)/2.0), static_cast<float>(min_z));

		std::cout << "total_x_length = " << total_x_length << std::endl;
		std::cout << "total_y_length = " << total_y_length << std::endl;
		std::cout << "total_z_length = " << total_z_length << std::endl;

		std::cout << "ori = (" << origin[0] << "," << origin[1] << "," << origin[2] << ")" << std::endl;
    }
};

#endif // MARCHING_CUBE_FLUID

