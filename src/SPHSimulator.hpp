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
#include "sim_record.hpp"

using namespace Simulator;

class SPHSimulator
{
public:
	const RealVector3 gravity = RealVector3(0.0, 0.0, -0.98);
	const Real dt;

    SPHSimulator(float neighbor_search_radius, Real dt, int N);
	SPHSimulator(float neighbor_search_radius, Real dt);

    void 					 generate_particles();
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




    /*-----use cereal output particles to json file-----*/

    void set_file_path(std::string fp)
    {
        file_path=fp;
    }


    void output_particles_xml()
    {
        std::ofstream myfile(file_path+"sim"+std::to_string(file_count)+".xml");   //maybe here, this ofstream could be useful... it should has a continuse read mode!
        file_count++;
        cereal::XMLOutputArchive output(myfile); // stream to cout
        output(CEREAL_NVP(particles));  //not good... maybe directly ar the vector
    }



    void input_particles_xml()
    {
        particles.clear();
        std::ifstream myfile(file_path+"sim"+std::to_string(file_count)+".xml");
        file_count++;
        cereal::XMLInputArchive input(myfile); // stream to cout
        input(CEREAL_NVP(particles));  //not good... maybe directly ar the vector
    }

    void update_sim_record_state()
    {
        SimulationState sim_state;
        sim_state.particles = particles;
        sim_rec.states.push_back(sim_state);
    }

    void output_sim_record_xml(std::string fp)
    {
        std::ofstream file(fp);
        cereal::XMLOutputArchive output(file); // stream to cout
        output(sim_rec);  //not good... maybe directly ar the vector
    }

    void output_sim_record_bin(std::string fp)
    {
        std::ofstream file(fp);
        cereal::BinaryOutputArchive output(file); // stream to cout
        output(sim_rec);  //not good... maybe directly ar the vector
    }

    /*-----use cereal read particles from json file-----*/
    //void input_particles_json(std::string file_path);
    /*-----This method should in the visualization class----*/

    void print_all_particles()
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



private:
	NeighborSearcher neighborSearcher;
	KernelHandler 	 kernelHandler;
	ParticleFunc 	 particleFunc;

	std::vector<RealVector3> positions;
	std::vector<mParticle> particles;
	size_t N;
	size_t number_of_particles;
	Real particle_radius;

	float neighbor_search_radius;
    int index_of_source_particle;
    std::vector<size_t> neighbors;

    /*----------this is for cereal-------------*/
    std::string file_path;
    unsigned int file_count;

    SimulationRecord sim_rec;

//    friend class cereal::access;
//    template <class Archive>
//    void serialize( Archive & ar )
//    {
//        ar(particles);
//    }
    /*-----------cereal end-------------------*/


    //std::array<RealVector3 sweep_line{{ RealVector3(-2.0, 0.0, 0.0), RealVector3(2.0, 0.0, 0.0) }};

	//void generate_random_particles();
	//void randomly_generate_celling_particles();
	void generate_celling_particles_at_center(Eigen::Ref<RealVector3> origin, bool do_clear, Eigen::Ref<RealVector3> v0);

	void generate_two_colliding_cubes();
	void generate_two_freefall_cubes();

};
