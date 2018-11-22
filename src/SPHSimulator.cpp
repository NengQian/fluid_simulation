#include "SPHSimulator.hpp"

#include "math_types.hpp"
#include "Particle.hpp"

#include <merely3d/merely3d.hpp>
#include <CompactNSearch/CompactNSearch>

#include <random>

using merely3d::renderable;
using merely3d::Rectangle;
using merely3d::Box;
using merely3d::Line;
using merely3d::Color;
using merely3d::Material;
using merely3d::Sphere;
using merely3d::red;
using merely3d::Particle;

using Eigen::AngleAxisf;
using Eigen::Vector3f;

/*
SPHSimulator::SPHSimulator(float neighbor_search_radius, Real dt) : dt(dt), kernelHandler(static_cast<Real>(neighbor_search_radius)), neighborSearcher(static_cast<Real>(neighbor_search_radius))
{
	set_N(5);
	set_particle_radius(1.0/N);
	generate_particles();
	set_neighbor_search_radius(neighbor_search_radius);
	set_index_of_source_particle(0);
}
*/

//SPHSimulator::SPHSimulator(float neighbor_search_radius, Real dt, int N) : dt(dt), kernelHandler(static_cast<Real>(neighbor_search_radius)), neighborSearcher(static_cast<Real>(neighbor_search_radius))
//{
//    set_N(N);
//    set_particle_radius(1.0/N);
//    generate_particles();
//    set_neighbor_search_radius(neighbor_search_radius);
//    set_index_of_source_particle(0);
//    file_count = 0;
//}

SPHSimulator::SPHSimulator(Real dt, int N) : dt(dt)
{
    set_N(N);
    set_particle_radius(1.0/N);
    generate_particles();
    set_neighbor_search_radius(2.4/N*2);
    set_index_of_source_particle(0);

    set_boundary_volumes();

    sim_rec.timestep = dt;
    update_sim_record_state();
}

void SPHSimulator::set_boundary_volumes()
{
	if (!boundary_volumes.empty())
		boundary_volumes.clear();

	boundary_volumes = particleFunc.initialize_boundary_particle_volumes(boundary_positions, static_cast<Real>(neighbor_search_radius));
}


void SPHSimulator::generate_particles()
{
	if (!particles.empty())
		particles.clear();

	if (!positions.empty())
		positions.clear();

	if (!boundary_particles.empty())
		boundary_particles.clear();

	if (!boundary_positions.empty())
		boundary_positions.clear();

	RealVector3 zero(0.0, 0.0, 0.0);

	particleGenerator.generate_rigid_box(boundary_particles, 3*N);
	particleGenerator.generate_cube(particles, N, zero, zero, zero);

	//particleGenerator.generate_two_colliding_cubes(particles, N);

	set_positions();
	set_boundary_positions();

	neighborSearcher.set_particles_ptr(positions);
	neighborSearcher.set_boundary_particles_ptr(boundary_positions);
}

void SPHSimulator::set_boundary_positions()
{
	if (!boundary_positions.empty())
		boundary_positions.clear();

	for (auto& bp : boundary_particles)
		boundary_positions.push_back(RealVector3(bp.position));
}

std::vector<RealVector3> SPHSimulator::get_boundary_positions()
{
	return boundary_positions;
}

void SPHSimulator::set_positions()
{
	if (!positions.empty())
		positions.clear();

	for (auto& p : particles)
		positions.push_back(RealVector3(p.position));
}

std::vector<RealVector3> SPHSimulator::get_positions()
{
	update_positions();
	return positions;
}

void SPHSimulator::set_particle_radius(Real r)
{
	particle_radius = r;
}

Real SPHSimulator::get_particle_radius()
{
	return particle_radius;
}

void   SPHSimulator::set_N(size_t n)
{
	N = n;
	set_number_of_particles(n * n * n);
}

size_t SPHSimulator::get_N()
{
	return N;
}

void SPHSimulator::set_number_of_particles(size_t n)
{
	number_of_particles = n;
}

size_t SPHSimulator::get_number_of_particles()
{
	return number_of_particles;
}

void SPHSimulator::set_neighbor_search_radius(float r)
{
	neighbor_search_radius = r;
	Real rr = static_cast<Real>(r);
	neighborSearcher.set_neighbor_search_radius(rr);
	kernelHandler.set_neighbor_search_radius(rr);
}

float SPHSimulator::get_neighbor_search_radius()
{
	return neighbor_search_radius;
}

void SPHSimulator::set_index_of_source_particle(int idx)
{
	index_of_source_particle = idx;
}

int SPHSimulator::get_index_of_source_particle()
{
	return index_of_source_particle;
}

void SPHSimulator::set_neighbors(std::vector<size_t>& ns)
{
	neighbors = ns;
}

std::vector<size_t> SPHSimulator::get_neighbors()
{
	return neighbors;
}

void SPHSimulator::find_and_set_neighbors(bool do_compactN)
{
	neighbors = neighborSearcher.find_neighbors_within_radius( index_of_source_particle, do_compactN );
}

std::vector< std::vector<size_t> > SPHSimulator::find_neighbors_of_all(bool do_compactN)
{
	return neighborSearcher.find_neighbors_within_radius( do_compactN );
}


Real SPHSimulator::compute_average_error_of_kernel_gradient(int kernel_type)
{
	Real error = 0;
	for (size_t i=0; i < neighbors.size(); ++i)
	{
		error += kernelHandler.test_gradient(positions[index_of_source_particle], positions[neighbors[i]], kernel_type);
	}

	return error/neighbors.size();
}

void SPHSimulator::sample_density()
{
	std::vector<mParticle> sample_particles;
	std::vector<RealVector3> sample_positions;
	size_t number_of_samples = N*10;

	for (size_t i=0; i<number_of_samples; ++i)
	{
		mParticle p;
		RealVector3 vec(-2.0 + i * 4.0 / number_of_samples, 0.0, 0.0);
		p.position = vec;
		sample_particles.push_back(p);
		sample_positions.push_back(vec);
	}

	set_neighbor_search_radius( 2.4/N * 2 );

	std::vector< std::vector<size_t> > neighbors_of_samples = neighborSearcher.find_neighbors_within_radius(sample_positions);

	std::vector<Real> densities;
	Real r = static_cast<Real>(neighbor_search_radius);
	densities = particleFunc.update_density(neighbors_of_samples, sample_particles, particles, r);

	for (size_t i=0; i<neighbors_of_samples.size(); ++i)
	{
		std::cout << sample_particles[i].position[0] << " " << densities[i] << std::endl;
	}
}


void SPHSimulator::update_positions()
{
	for (size_t i=0; i<particles.size(); ++i)
	{
		positions[i][0] = particles[i].position[0];
		positions[i][1] = particles[i].position[1];
		positions[i][2] = particles[i].position[2];
	}
	neighborSearcher.set_particles_ptr(positions);
}

void SPHSimulator::update_freefall_motion()
{
	for (auto& p : particles)
	{
		p.acceleration = gravity;
	}
	particleFunc.update_velocity(particles, dt);
	particleFunc.update_position(particles, dt);

	update_positions();
}

void SPHSimulator::update_two_cubes_collision()
{
	set_neighbor_search_radius( 2.4/N * 2 );
	std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);

	/*
	for (size_t i=0; i<neighbors_set.size(); ++i)
	{
		std::cout << i << std::endl;

		for (size_t j=0; j<neighbors_set[i].size(); ++j)
			std::cout << neighbors_set[i][j] << " ";

		std::cout << std::endl;
	}
	*/

	Real r = static_cast<Real>(neighbor_search_radius);
	std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

	Real water_rest_density = 1000.0;
	Real B = 1000.0;

	std::vector<RealVector3> external_forces;
	for (size_t i=0; i<particles.size(); ++i)
		external_forces.push_back( RealVector3(0.0, 0.0, 0.0) );

	particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);
	particleFunc.update_velocity(particles, dt);
	particleFunc.update_position(particles, dt);

	update_positions();
}

void SPHSimulator::update_rigid_body_simulation()
{
	float r = 2.4/N * 2 ;
	set_neighbor_search_radius(r);

	std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
	std::vector< std::vector<size_t> > neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

/*
	for (size_t i=0; i<neighbors_set.size(); ++i)
	{
		std::cout << i << std::endl;

		for (size_t j=0; j<neighbors_set[i].size(); ++j)
			std::cout << neighbors_set[i][j] << " ";

		std::cout << std::endl;
	}
*/
	std::vector<Real> densities = particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, boundary_volumes, r);
	//std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

	Real water_rest_density = 1000.0;
	Real B = 100.0;

	std::vector<RealVector3> external_forces;
	for (size_t i=0; i<particles.size(); ++i)
		external_forces.push_back( RealVector3(0.0, 0.0, -0.981 * particles[i].mass) );

	particleFunc.update_acceleration( particles, boundary_particles, neighbors_set, neighbors_in_boundary, densities, boundary_volumes, external_forces, water_rest_density, r, B);
	//particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);

	particleFunc.update_velocity(particles, dt);
	particleFunc.update_position(particles, dt);

	update_positions();
}

void SPHSimulator::update_sim_record_state()
{
    SimulationState sim_state;
    sim_state.particles = particles;
    sim_rec.states.push_back(sim_state);
}

void SPHSimulator::output_sim_record_bin(std::string fp)
{
    std::ofstream file(fp);
    cereal::BinaryOutputArchive output(file); // stream to cout
    output(sim_rec);  //not good... maybe directly ar the vector
}



void SPHSimulator::print_all_particles()
{
    std::cout<<"now print particles set, its size is "<<particles.size()<<std::endl;
    for(size_t i=0;i<particles.size();i++)
    {
        mParticle particle(particles[i]);
        std::cout<<"position is"<< std::endl<<particle.position<<std::endl;
        std::cout<<"velocity is"<<  std::endl<<particle.velocity<<std::endl;
        std::cout<<"acceleration is"<< std::endl<<particle.acceleration<<std::endl;
        std::cout<<"mass is"<< std::endl<<particle.mass<<std::endl;
    }
}
