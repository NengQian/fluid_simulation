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

class ParticleGenerator {
public:
    void generate_cube(std::vector<mParticle>& particles, size_t N,
                       Eigen::Ref<RealVector3> origin,
                       Eigen::Ref<RealVector3> v0,
                       Real halfExtent=1.0,
                       bool do_clear=true,
                       bool hollow=false);
    void generate_two_colliding_cubes(std::vector<mParticle>& particles, size_t N, Real radius);
    void generate_two_freefall_cubes(std::vector<mParticle>& particles, size_t N, Real radius);
    void generate_rigid_box(std::vector<mParticle>& particles, size_t N, Real radius);
    void generate_cuboid_box(std::vector<mParticle>& particles,
                             Eigen::Ref<RealVector3> v0,
                             mCuboid cuboid,   // each cuboid contains number of particles per x,y,z. and also origin;
                             Real radius,
                             bool do_clear=false);

private:
};
