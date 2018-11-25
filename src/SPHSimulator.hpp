#pragma once
#include <sstream>
#include <iostream>
#include <fstream>
#include <string.h>
#include <Eigen/Geometry>
#include <CompactNSearch/CompactNSearch>
#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include "math_types.hpp"
#include "NeighborSearcher.hpp"
#include "KernelHandler.hpp"
#include "Particle.hpp"
#include "ParticleFunc.hpp"
#include "ParticleGenerator.hpp"
#include "sim_record.hpp"

using namespace Simulator;

class SPHSimulator
{
public:
    const RealVector3 gravity = RealVector3(0.0, 0.0, -15.0);
	const Real dt;

    //SPHSimulator(Real dt, int N=5);
    //SPHSimulator(Real particle_radius, std::vector<Real>& cuboid_side_lengths, Real dt=0.01, Real eta=1.2, Real B=100.0, Real rest_density=1000.0, Real alpha=0.08, int kernel_type=4);
    SPHSimulator(int N, Real dt=0.01, Real eta=1.2, Real B=100.0, Real alpha=0.08, Real rest_density=1000.0);


    std::vector<RealVector3> get_boundary_positions();
    std::vector<RealVector3> get_positions();
    int get_N();
    Real get_particle_radius();

    void set_boundary_positions();
    void set_boundary_volumes();
    void set_positions();
    void set_particle_radius(Real r);
    void set_N(size_t n);
    void set_neighbor_search_radius(float r);

	void sample_density();


/*----------virtual function-----------------*/
    virtual void update_simulation();
    virtual void generate_particles();
    virtual ~SPHSimulator() = default;
/*-------------------------------------------*/

    void update_positions();
    //void update_freefall_motion();
    //void update_two_cubes_collision();
    //void update_rigid_body_simulation();

    /*-----use cereal output particles to json file-----*/
    void update_sim_record_state();
    void output_sim_record_bin(std::string fp);
    void print_all_particles();
   /*------cereal task over----------------------------*/

protected:
	NeighborSearcher neighborSearcher;
	KernelHandler 	 kernelHandler;
	ParticleFunc 	 particleFunc;
	ParticleGenerator particleGenerator;

    std::vector<RealVector3> positions;   //why we need this positions... neng
	std::vector<RealVector3> boundary_positions;

	std::vector<Real> boundary_volumes;

	std::vector<mParticle> particles;
	std::vector<mParticle> boundary_particles;

	size_t N;
	Real particle_radius;
	std::vector<Real> cuboid_shape;

	float neighbor_search_radius;

    /*----------this is for cereal-------------*/
    SimulationRecord sim_rec;
};
