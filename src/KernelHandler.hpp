#pragma once

#include "math_types.hpp"

#include <Eigen/Geometry>

using namespace Simulator;

class KernelHandler
{
public:
	KernelHandler();

	Real compute_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension );
	Real compute_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension );
	Real compute_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension );

	RealVectorX gradient_of_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, bool analytical_solution=true, Real epsilon=0.000001 );
	RealVectorX gradient_of_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, bool analytical_solution=true, Real epsilon=0.000001 );
	RealVectorX gradient_of_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, bool analytical_solution=true, Real epsilon=0.000001 );

	Real test_M4_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, Real epsilon=0.000001 );
	Real test_M5_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, Real epsilon=0.000001 );
	Real test_M6_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, Real epsilon=0.000001 );

	Real integrate_M4_kernel(Real h, int number_of_dimension);

private:

};
