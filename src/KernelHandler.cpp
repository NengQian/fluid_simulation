#include "KernelHandler.hpp"
#include "math_types.hpp"

#include <algorithm>    // std::sort
#include <numeric>
#include <cmath>
#include <iostream>

using namespace Simulator;

KernelHandler::KernelHandler()
{

}

KernelHandler::KernelHandler(Real radius)
{
	set_neighbor_search_radius(radius);
}

void KernelHandler::set_neighbor_search_radius(Real radius)
{
	neighbor_search_radius = radius;
}

Real KernelHandler::compute_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, int kernel_type )
{
	switch (kernel_type) {
	case 4:
		return compute_M4_kernel(source_particle, destination_particle);
	case 5:
		return compute_M5_kernel(source_particle, destination_particle);
	case 6:
		return compute_M6_kernel(source_particle, destination_particle);
	default:
		throw "Invalid kernel type!";
	}
}

RealVectorX KernelHandler::gradient_of_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, int kernel_type, bool analytical_solution )
{
	if (analytical_solution)
	{
		switch (kernel_type) {
		case 4:
			return gradient_of_M4_kernel(source_particle, destination_particle);
		case 5:
			return gradient_of_M5_kernel(source_particle, destination_particle);
		case 6:
			return gradient_of_M6_kernel(source_particle, destination_particle);
		default:
			throw "Invalid kernel type!";
		}
	} else {

		RealVectorX gradient_W_over_x(source_particle.size());
		RealVectorX zero_vector = RealVectorX::Zero(source_particle.size());

		for (size_t i=0; i<source_particle.size(); ++i)
		{
			RealVectorX unit_vector_in_ith_axis = zero_vector;
			unit_vector_in_ith_axis[i] = 1.0;

			RealVectorX shifted_right_source_particle = source_particle + epsilon * unit_vector_in_ith_axis;
			RealVectorX shifted_left_source_particle = source_particle - epsilon * unit_vector_in_ith_axis;

			gradient_W_over_x[i] = compute_kernel(shifted_right_source_particle , destination_particle, kernel_type) - compute_kernel(shifted_left_source_particle , destination_particle, kernel_type);
		}

		return gradient_W_over_x = gradient_W_over_x / (2.0 * epsilon);
	}

	throw "Something wrong in gradient_of_kernel!";
}

Real KernelHandler::test_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, int kernel_type )
{
	RealVectorX analytical_solution_of_gradient = gradient_of_kernel(source_particle, destination_particle, kernel_type, true);
	RealVectorX approximate_solution_of_gradient = gradient_of_kernel(source_particle, destination_particle, kernel_type, false);

	Real error_rate = (analytical_solution_of_gradient - approximate_solution_of_gradient).norm() / analytical_solution_of_gradient.norm();
	return error_rate;
}

Real KernelHandler::compute_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle )
{
	int number_of_dimension = source_particle.size();

	Real h = neighbor_search_radius / 2.0;
	Real q = (source_particle - destination_particle).norm() / h;

	Eigen::Vector3d sigma_list(2.0/3.0, 10.0/(7.0*(Real)M_PI), 1.0/(Real)M_PI);
	Real sigma = sigma_list[number_of_dimension - 1];

	Real W = 0.0;
	Real inside_w = 0.0;

	if (q >= 1.0 && q < 2.0)
	{
		inside_w = 1.0/4.0 * std::pow(2.0-q, 3.0);
	}
	else if (q >= 0.0 && q < 1.0)
	{
		inside_w = 1.0/4.0 * std::pow(2.0-q, 3.0) - std::pow(1.0-q, 3.0);
	}

	W = sigma * inside_w * 1.0/std::pow(h, number_of_dimension);

	return W;
}

Real KernelHandler::compute_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle )
{
	int number_of_dimension = source_particle.size();

	Real h = neighbor_search_radius / 2.5;
	Real q = (source_particle - destination_particle).norm() / h;

	Eigen::Vector3d sigma_list(1.0/24.0, 96.0/(1199.0*(Real)M_PI), 1.0/(20.*(Real)M_PI));
	Real sigma = sigma_list[number_of_dimension - 1];

	Real W = 0.0;
	Real inside_w = 0.0;

	if (q >= 1.5 && q < 2.5)
	{
		inside_w = std::pow(2.5-q, 4.0);
	}
	else if (q >= 0.5 && q < 1.5)
	{
		inside_w = std::pow(2.5-q, 4.0) - 5.0 * std::pow(1.5-q, 4.0);
	}
	else if (q >= 0.0 && q < 0.5)
	{
		inside_w = std::pow(2.5-q, 4.0) - 5.0 * std::pow(1.5-q, 4.0) + 10.0 * std::pow(0.5-q, 4.0);
	}

	W = sigma * inside_w * 1.0/std::pow(h, number_of_dimension);

	return W;
}

Real KernelHandler::compute_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle )
{
	int number_of_dimension = source_particle.size();

	Real h = neighbor_search_radius / 3.0;
	Real q = (source_particle - destination_particle).norm() / h;

	Eigen::Vector3d sigma_list(1.0/120.0, 7.0/(478.0*(Real)M_PI), 1.0/(120.*(Real)M_PI));
	Real sigma = sigma_list[number_of_dimension - 1];

	Real W = 0.0;
	Real inside_w = 0.0;

	if (q >= 2.0 && q < 3.0)
	{
		inside_w = std::pow(3.0-q, 5.0);
	}
	else if (q >= 1.0 && q < 2.0)
	{
		inside_w = std::pow(3.0-q, 5.0) - 6.0 * std::pow(2.0-q, 5.0);
	}
	else if (q >= 0.0 && q < 1.0)
	{
		inside_w = std::pow(3.0-q, 5.0) - 6.0 * std::pow(2.0-q, 5.0) + 15.0 * std::pow(1.0-q, 5.0);
	}

	W = sigma * inside_w * 1.0/std::pow(h, number_of_dimension);

	return W;
}

RealVectorX KernelHandler::gradient_of_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle )
{
	int number_of_dimension = source_particle.size();

	RealVectorX gradient_W_over_x(source_particle.size());

	Eigen::Vector3d sigma_list(2.0/3.0, 10.0/(7.0*(Real)M_PI), 1.0/(Real)M_PI);
	Real sigma = sigma_list[number_of_dimension - 1];

	Real h = neighbor_search_radius / 2.0;
	Real q = (source_particle - destination_particle).norm() / h;

	RealVectorX gradient_q_over_x = (source_particle-destination_particle) / (source_particle-destination_particle).norm() / h;
	Real gradient_w_over_q = 0.0;

	if (q >= 1.0 && q < 2.0)
	{
		gradient_w_over_q = sigma * (-3.0/4.0) * std::pow(2.0-q, 2.0);
	}
	else if (q >= 0.0 && q < 1.0)
	{
		gradient_w_over_q = sigma * ( (-3.0/4.0) * std::pow(2.0-q, 2.0) + 3.0 * std::pow(1.0-q, 2.0) );
	}

	gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, number_of_dimension) * gradient_q_over_x;

	return gradient_W_over_x;
}

RealVectorX KernelHandler::gradient_of_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle )
{
	int number_of_dimension = source_particle.size();

	RealVectorX gradient_W_over_x(source_particle.size());

	Eigen::Vector3d sigma_list(1.0/24.0, 96.0/(1199.0*(Real)M_PI), 1.0/(20.*(Real)M_PI));
	Real sigma = sigma_list[number_of_dimension - 1];

	Real h = neighbor_search_radius / 2.5;
	Real q = (source_particle - destination_particle).norm() / h;

	RealVectorX gradient_q_over_x = (source_particle-destination_particle) / (source_particle-destination_particle).norm() / h;
	Real gradient_w_over_q = 0.0;

	if (q >= 1.5 && q < 2.5)
	{
		gradient_w_over_q = sigma * (-4.0) * std::pow(2.5-q, 3.0);
	}
	else if (q >= 0.5 && q < 1.5)
	{
		gradient_w_over_q = sigma * ( (-4.0) * std::pow(2.5-q, 3.0) + 20.0 * std::pow(1.5-q, 3.0) );
	}
	else if (q >= 0.0 && q < 0.5)
	{
		gradient_w_over_q = sigma * ( (-4.0) * std::pow(2.5-q, 3.0) + 20.0 * std::pow(1.5-q, 3.0) - 40.0 * std::pow(0.5-q, 3.0) );
	}

	gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, number_of_dimension) * gradient_q_over_x;

	return gradient_W_over_x;
}

RealVectorX KernelHandler::gradient_of_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle )
{
	int number_of_dimension = source_particle.size();

	RealVectorX gradient_W_over_x(source_particle.size());

	Eigen::Vector3d sigma_list(1.0/120.0, 7.0/(478.0*(Real)M_PI), 1.0/(120.0*(Real)M_PI));
	Real sigma = sigma_list[number_of_dimension - 1];

	Real h = neighbor_search_radius / 3.0;
	Real q = (source_particle - destination_particle).norm() / h;

	RealVectorX gradient_q_over_x = (source_particle-destination_particle) / (source_particle-destination_particle).norm() / h;
	Real gradient_w_over_q = 0.0;

	if (q >= 2.0 && q < 3.0)
	{
		gradient_w_over_q = sigma * (-5.0) * std::pow(3.0-q, 4.0);
	}
	else if (q >= 1.0 && q < 2.0)
	{
		gradient_w_over_q = sigma * ( (-5.0) * std::pow(3.0-q, 4.0) + 30.0 * std::pow(2.0-q, 4.0) );
	}
	else if (q >= 0.0 && q < 1.0)
	{
		gradient_w_over_q = sigma * ( (-5.0) * std::pow(3.0-q, 4.0) + 30.0 * std::pow(2.0-q, 4.0) - 75.0 * std::pow(1.0-q, 4.0) );
	}

	gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;

	return gradient_W_over_x;
}

Real KernelHandler::integrate_kernel( int kernel_type, int number_of_dimension )
{
	RealVectorX origin = RealVectorX::Zero(number_of_dimension);

	Real integration = 0.0;

	Real step_size = 2 * neighbor_search_radius / 100.0;

	RealVectorX dest = RealVectorX::Zero(number_of_dimension);

	for (Real i = (-1.0)*neighbor_search_radius; i <= neighbor_search_radius; i += step_size)
	{
		dest[0] = i;
		if (number_of_dimension == 1)
			integration += compute_kernel( origin, dest, kernel_type ) * step_size;
		else {
			for (Real j = (-1.0)*neighbor_search_radius; j <= neighbor_search_radius; j += step_size)
			{
				dest[1] = j;
				if (number_of_dimension == 2)
					integration += compute_kernel( origin, dest, kernel_type ) * step_size * step_size;
				else {
					for (Real k = (-1.0)*neighbor_search_radius; k <= neighbor_search_radius; k += step_size)
					{
						dest[2] = k;
						if (number_of_dimension == 3)
							integration += compute_kernel( origin, dest, kernel_type ) * step_size * step_size * step_size;
						else
							throw "Invalid number of dimension";
					}
				}
			}
		}
	}

	return integration;
}
