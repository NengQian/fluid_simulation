#pragma once

#include "math_types.hpp"

#include <Eigen/Geometry>

using namespace Simulator;

class KernelHandler
{
public:
	KernelHandler();
	KernelHandler(Real radius);

	void set_neighbor_search_radius(Real radius);

	Real compute_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, int kernel_type );
	RealVectorX gradient_of_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, int kernel_type, bool analytical_solution=true );
	Real test_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, int kernel_type );
	Real integrate_kernel( int kernel_type, int number_of_dimension );

private:
	Real neighbor_search_radius = 5.0;
	Real epsilon = 0.000001;

	Real compute_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle );
	Real compute_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle );
	Real compute_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle );

	RealVectorX gradient_of_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle );
	RealVectorX gradient_of_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle );
	RealVectorX gradient_of_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle );
};
