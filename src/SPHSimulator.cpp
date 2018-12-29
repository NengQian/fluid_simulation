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

SPHSimulator::SPHSimulator(int N, Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int solver_type) : dt(dt), //
																				  rest_density(rest_density), //
																				  N(N), //
																				  particleFunc(rest_density, B, alpha), //
																				  solver_type(solver_type) //
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
void SPHSimulator::set_boundary_attribute()
{
	std::vector<Real> boundary_volumes;
	particleFunc.initialize_boundary_particle_volumes(boundary_volumes, boundary_positions, neighbor_search_radius);

	// set mass of boundary particle, m = rest_density * volume
	for (size_t i=0; i<boundary_particles.size(); ++i)
	{
		boundary_particles[i].mass = rest_density * boundary_volumes[i];
	}
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
        std::cout<<"density is"<< std::endl<<particle.density<<std::endl;
        std::cout<<"mass is"<< std::endl<<particle.mass<<std::endl;
    }
}
