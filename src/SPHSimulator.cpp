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

SPHSimulator::SPHSimulator(int N, Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density) : dt(dt), //
																				  N(N), //
																				  particleFunc(rest_density, B, alpha) //
{
	set_particle_radius(uParticle_len/2);  // uParticle_len = 2.0 * radius
    Real h = eta*uParticle_len;   // h = eta*uParticle_len , in paper it is h = eta*(m/rou)^(1/3). while m/rou = v = uParticle_len^3;
    set_neighbor_search_radius(2.0*h);  // and also since we are using cubic .. so the radius is 2.0 * h...

    sim_rec.timestep = dt;
    sim_rec.unit_particle_length = uParticle_len;
    sim_rec.boundary_particles.clear();
    cuboids.clear();
    //update_sim_record_state();
}
/*
SPHSimulator::SPHSimulator(Real particle_radius, std::vector<Real>& cuboid_side_lengths, Real dt, Real eta, Real B, Real rest_density, int kernel_type) : dt(dt), //
																																					 	  cuboid_shape(cuboid_side_lengths), //
																																						  particle_radius(particle_radius),
																																						  kernelHandler(kernel_type), //
																																						  particleFunc(rest_density, B), //
{
	    set_N(N);
	    set_particle_radius(1.0/N);
	    generate_particles();
	    set_neighbor_search_radius(2.4/N*2);
	    set_boundary_volumes();

	    sim_rec.timestep = dt;
	    update_sim_record_state();
}
*/
void SPHSimulator::set_boundary_volumes()
{
	particleFunc.initialize_boundary_particle_volumes(boundary_volumes, boundary_positions, neighbor_search_radius);
}

void SPHSimulator::generate_particles()
{

}

void SPHSimulator::set_boundary_positions()
{
    if (!boundary_positions.empty())
        boundary_positions.clear();

    for (auto& bp : boundary_particles)
        boundary_positions.push_back(RealVector3(bp.position));
}

const std::vector<RealVector3>& SPHSimulator::get_boundary_positions() const
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

const std::vector<RealVector3>& SPHSimulator::get_positions() const
{
	//update_positions();
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

void SPHSimulator::set_N(size_t n)
{
	N = n;
}

int SPHSimulator::get_N()
{
	return N;
}

void SPHSimulator::set_neighbor_search_radius(Real r)
{
	neighbor_search_radius = r;
	neighborSearcher.set_neighbor_search_radius(r);
	kernelHandler.set_neighbor_search_radius(r);
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


void SPHSimulator::update_simulation()
{
}


//void SPHSimulator::update_freefall_motion()
//{
//	for (auto& p : particles)
//	{
//		p.acceleration = gravity;
//	}
//	particleFunc.update_velocity(particles, dt);
//	particleFunc.update_position(particles, dt);

//	update_positions();
//}

//void SPHSimulator::update_two_cubes_collision()
//{
//	set_neighbor_search_radius( 2.4/N * 2 );
//	std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);

//	/*
//	for (size_t i=0; i<neighbors_set.size(); ++i)
//	{
//		std::cout << i << std::endl;

//		for (size_t j=0; j<neighbors_set[i].size(); ++j)
//			std::cout << neighbors_set[i][j] << " ";
/*
<<<<<<< HEAD
	std::vector<RealVector3> external_forces;
	for (size_t i=0; i<particles.size(); ++i)
		external_forces.push_back( RealVector3(0.0, 0.0, 0.0) );

	particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, r);
	particleFunc.update_velocity(particles, dt);
	particleFunc.update_position(particles, dt);
=======
*/
//		std::cout << std::endl;
//	}
//	*/

//	Real r = static_cast<Real>(neighbor_search_radius);
//	std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

//	Real water_rest_density = 1000.0;
//    Real B = 1000.0;
//>>>>>>> neng3

//	std::vector<RealVector3> external_forces;
//	for (size_t i=0; i<particles.size(); ++i)
//		external_forces.push_back( RealVector3(0.0, 0.0, 0.0) );

//	particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);
//	particleFunc.update_velocity(particles, dt);
//	particleFunc.update_position(particles, dt);

//	update_positions();
//}

//void SPHSimulator::update_rigid_body_simulation()
//{
//	float r = 2.4/N * 2 ;
//	set_neighbor_search_radius(r);

//	std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
//	std::vector< std::vector<size_t> > neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

///*
//	for (size_t i=0; i<neighbors_set.size(); ++i)
//	{
//		std::cout << i << std::endl;
/*
<<<<<<< HEAD
	std::vector<RealVector3> external_forces;
	for (size_t i=0; i<particles.size(); ++i)
		external_forces.push_back( RealVector3(0.0, 0.0, -0.981 * particles[i].mass) );

	particleFunc.update_acceleration( particles, boundary_particles, neighbors_set, neighbors_in_boundary, densities, boundary_volumes, external_forces, r);
	//particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);
=======
*/
//		for (size_t j=0; j<neighbors_set[i].size(); ++j)
//			std::cout << neighbors_set[i][j] << " ";

//		std::cout << std::endl;
//	}
//*/
//	std::vector<Real> densities = particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, boundary_volumes, r);
//	//std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

//	Real water_rest_density = 1000.0;
//    Real B = 100.0;
//>>>>>>> neng3

//	std::vector<RealVector3> external_forces;
//	for (size_t i=0; i<particles.size(); ++i)
//        //external_forces.push_back( RealVector3(0.0, 0.0, -0.981 * particles[i].mass) ); //Neng: we have the gravity in class private
//        external_forces.push_back( gravity * particles[i].mass ); //Neng: we have the gravity in class private

//	particleFunc.update_acceleration( particles, boundary_particles, neighbors_set, neighbors_in_boundary, densities, boundary_volumes, external_forces, water_rest_density, r, B);
//	//particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);

//	particleFunc.update_velocity(particles, dt);
//	particleFunc.update_position(particles, dt);

//	update_positions();
//}

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
