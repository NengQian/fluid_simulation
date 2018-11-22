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
	const RealVector3 gravity = RealVector3(0.0, 0.0, -0.98);
	const Real dt;

    SPHSimulator(Real dt, int N=5);
	//SPHSimulator(float neighbor_search_radius, Real dt);

    void generate_particles();

    void set_boundary_positions();
    std::vector<RealVector3> get_boundary_positions();

    void set_boundary_volumes();

    void set_positions();
    std::vector<RealVector3> get_positions();

    void set_particle_radius(Real r);
    Real get_particle_radius();

    void   set_N(size_t n);
    size_t get_N();

    void   set_number_of_particles(size_t n);
    size_t get_number_of_particles();

    void  set_neighbor_search_radius(float r);
    float get_neighbor_search_radius();

    void set_index_of_source_particle(int idx);
    int  get_index_of_source_particle();

    void 			    set_neighbors(std::vector<size_t>& ns);
    std::vector<size_t> get_neighbors();

	void 							 find_and_set_neighbors(bool do_compactN);
	std::vector<std::vector<size_t>> find_neighbors_of_all(bool do_compactN);

	Real compute_average_error_of_kernel_gradient(int kernel_type);

	void sample_density();

	void update_positions();
	void update_freefall_motion();
	void update_two_cubes_collision();
	void update_rigid_body_simulation();

    /*-----use cereal output particles to json file-----*/
    void update_sim_record_state();
    void output_sim_record_bin(std::string fp);
    void print_all_particles();
   /*------cereal task over----------------------------*/

private:
	NeighborSearcher neighborSearcher;
	KernelHandler 	 kernelHandler;
	ParticleFunc 	 particleFunc;
	ParticleGenerator particleGenerator;

	std::vector<RealVector3> positions;
	std::vector<RealVector3> boundary_positions;

	std::vector<Real> boundary_volumes;

	std::vector<mParticle> particles;
	std::vector<mParticle> boundary_particles;

	size_t N;
	size_t number_of_particles;
	Real particle_radius;

	float neighbor_search_radius;
    int index_of_source_particle;
    std::vector<size_t> neighbors;

    /*----------this is for cereal-------------*/
    SimulationRecord sim_rec;
};
