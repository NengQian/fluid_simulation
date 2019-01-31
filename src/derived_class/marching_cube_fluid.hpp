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


    void update_marching_cube()
    {
    	load_next_particles();
    	if (end)
    		return;

		update_grid_size();
		pick_up_particles();
        initialize_vertices();

    	compute_vertices_phi(); // for each vertices, compute its phi value
    	mark_vertices();
        initialize_edges();
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


    void load_next_particles()
    {
        if (count >= particles_series.size())
        {
            end = true;
            return;
        }

        current_particles = particles_series[count];
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
    }

    void update_voxel()
    {
        voxelx_n = static_cast<size_t>(ceil(total_x_length/unit_voxel_length));
        voxely_n = static_cast<size_t>(ceil(total_y_length/unit_voxel_length));
        voxelz_n = static_cast<size_t>(ceil(total_z_length/unit_voxel_length));

        voxel_verticesx_n = voxelx_n+1;
        voxel_verticesy_n = voxely_n+1;
        voxel_verticesz_n = voxelz_n+1;
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


    virtual void bitcode_to_mesh_vertices() override
    {
       marching_cube::bitcode_to_mesh_vertices();

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
            it->normal = Vector3f(0.0f, 0.0f, 0.0f);
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


    void save_grid_position()
    {

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
    	}
    }




    virtual void compute_vertex_normal(const Vector3f& vertex, Vector3f& normal) override
    {
        // since we cannot directly use this to compute know, so we just override and make it empty,
    }

    virtual void compute_vertices_phi() override
    {
        update_particle_positions();
        save_grid_position(); // for compactNsearch. since compactNsearch need a vector of points position as input
        std::vector<std::vector<size_t>> neighbor_indices = ns.find_neighbors_within_radius(grid_position);

        // recompute density, ignore boundary particles
        std::vector< std::vector<size_t> > tmp_n = ns.find_neighbors_within_radius( true );

        pf.update_density(tmp_n, current_particles, search_radius);

        size_t len = voxel_vertices.size();
        for (size_t i=0; i<len; ++i)
        {
            RealVector3 grid_vertex = grid_position[i];

            voxel_vertices[i].phi = -c;


            for (size_t j=0; j<neighbor_indices[i].size(); ++j)
            {


                int idx = neighbor_indices[i][j];

                mParticle p = current_particles[idx];

                voxel_vertices[i].phi += p.mass / p.density * kh.compute_kernel(grid_vertex, p.position);


            }

        }

        return;
    }




};

#endif // MARCHING_CUBE_FLUID

