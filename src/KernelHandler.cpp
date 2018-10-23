#include "KernelHandler.hpp"

#include "math_types.hpp"

#include <merely3d/merely3d.hpp>

#include <algorithm>    // std::sort
#include <numeric>
#include <cmath>

#include <iostream>
#include <typeinfo>

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

using namespace Simulator;

KernelHandler::KernelHandler()
{

}

Real KernelHandler::compute_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension )
{
	int m_number_of_dimension = number_of_dimension - 1;

	Real h = neighbor_search_radius / 2.0;
	Real q = (source_particle - destination_particle).norm() / h;

	Eigen::Vector3d sigma_list(2.0/3.0, 10.0/(7.0*(Real)M_PI), 1.0/(Real)M_PI);
	Real sigma = sigma_list[m_number_of_dimension];

	Real W = 0.0;

	if (q >= 1.0 && q < 2.0)
	{
		Real inside_w = 1.0/4.0 * std::pow(2.0-q, 3.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}
	else if (q >= 0.0 && q < 1.0)
	{
		Real inside_w = 1.0/4.0 * std::pow(2.0-q, 3.0) - std::pow(1.0-q, 3.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}

	return W;
}

Real KernelHandler::compute_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension )
{
	int m_number_of_dimension = number_of_dimension - 1;

	Real h = neighbor_search_radius / 2.5;
	Real q = (source_particle - destination_particle).norm() / h;

	Eigen::Vector3d sigma_list(1.0/24.0, 96.0/(1199.0*(Real)M_PI), 1.0/(20.*(Real)M_PI));
	Real sigma = sigma_list[m_number_of_dimension];

	Real W = 0.0;

	if (q >= 1.5 && q < 2.5)
	{
		Real inside_w = std::pow(2.5-q, 4.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}
	else if (q >= 0.5 && q < 1.5)
	{
		Real inside_w = std::pow(2.5-q, 4.0) - 5.0 * std::pow(1.5-q, 4.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}
	else if (q >= 0.0 && q < 0.5)
	{
		Real inside_w = std::pow(2.5-q, 4.0) - 5.0 * std::pow(1.5-q, 4.0) + 10.0 * std::pow(0.5-q, 4.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}
	return W;
}

Real KernelHandler::compute_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension )
{
	int m_number_of_dimension = number_of_dimension - 1;

	Real h = neighbor_search_radius / 3.0;
	Real q = (source_particle - destination_particle).norm() / h;

	Eigen::Vector3d sigma_list(1.0/120.0, 7.0/(478.0*(Real)M_PI), 1.0/(120.*(Real)M_PI));
	Real sigma = sigma_list[m_number_of_dimension];

	Real W = 0.0;

	if (q >= 2.0 && q < 3.0)
	{
		Real inside_w = std::pow(3.0-q, 5.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}
	else if (q >= 1.0 && q < 2.0)
	{
		Real inside_w = std::pow(3.0-q, 5.0) - 6.0 * std::pow(2.0-q, 5.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}
	else if (q >= 0.0 && q < 1.0)
	{
		Real inside_w = std::pow(3.0-q, 5.0) - 6.0 * std::pow(2.0-q, 5.0) + 15.0 * std::pow(1.0-q, 5.0);
		W = sigma * inside_w * 1.0/std::pow(h, 3.0);
	}
	return W;
}

RealVectorX KernelHandler::gradient_of_M4_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, bool analytical_solution, Real epsilon )
{
	int m_number_of_dimension = number_of_dimension - 1;

	RealVectorX gradient_W_over_x(source_particle.size());

	Eigen::Vector3d sigma_list(2.0/3.0, 10.0/(7.0*(Real)M_PI), 1.0/(Real)M_PI);
	Real sigma = sigma_list[m_number_of_dimension];

	if (analytical_solution)
	{
		Real h = neighbor_search_radius / 2.0;
		Real q = (source_particle - destination_particle).norm() / h;

		RealVectorX gradient_q_over_x = (source_particle-destination_particle) / (source_particle-destination_particle).norm() / h;

		if (q >= 1.0 && q < 2.0)
		{
			Real gradient_w_over_q = sigma * (-3.0/4.0) * std::pow(2.0-q, 2.0);
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
		else if (q >= 0.0 && q < 1.0)
		{
			Real gradient_w_over_q = sigma * ( (-3.0/4.0) * std::pow(2.0-q, 2.0) + 3.0 * std::pow(1.0-q, 2.0) );
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
	} else {
		RealVectorX zero_vector = RealVectorX::Zero(source_particle.size());
		for (size_t i=0; i<source_particle.size(); ++i)
		{
			RealVectorX unit_vector_in_ith_axis = zero_vector;
			unit_vector_in_ith_axis[i] = 1.0;

			RealVectorX shifted_right_source_particle = source_particle + epsilon * unit_vector_in_ith_axis;
			RealVectorX shifted_left_source_particle = source_particle - epsilon * unit_vector_in_ith_axis;

			gradient_W_over_x[i] = compute_M4_kernel(shifted_right_source_particle , destination_particle, neighbor_search_radius, number_of_dimension) - compute_M4_kernel(shifted_left_source_particle , destination_particle, neighbor_search_radius, number_of_dimension);
		}

		gradient_W_over_x = gradient_W_over_x / (2.0 * epsilon);
	}

	return gradient_W_over_x;
}

RealVectorX KernelHandler::gradient_of_M5_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, bool analytical_solution, Real epsilon )
{
	int m_number_of_dimension = number_of_dimension - 1;

	RealVectorX gradient_W_over_x(source_particle.size());

	Eigen::Vector3d sigma_list(1.0/24.0, 96.0/(1199.0*(Real)M_PI), 1.0/(20.*(Real)M_PI));
	Real sigma = sigma_list[m_number_of_dimension];

	if (analytical_solution)
	{
		Real h = neighbor_search_radius / 2.5;
		Real q = (source_particle - destination_particle).norm() / h;

		RealVectorX gradient_q_over_x = (source_particle-destination_particle) / (source_particle-destination_particle).norm() / h;

		if (q >= 1.5 && q < 2.5)
		{
			Real gradient_w_over_q = sigma * (-4.0) * std::pow(2.5-q, 3.0);
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
		else if (q >= 0.5 && q < 1.5)
		{
			Real gradient_w_over_q = sigma * ( (-4.0) * std::pow(2.5-q, 3.0) + 20.0 * std::pow(1.5-q, 3.0) );
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
		else if (q >= 0.0 && q < 0.5)
		{
			Real gradient_w_over_q = sigma * ( (-4.0) * std::pow(2.5-q, 3.0) + 20.0 * std::pow(1.5-q, 3.0) - 40.0 * std::pow(0.5-q, 3.0) );
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
	} else {
		RealVectorX zero_vector = RealVectorX::Zero(source_particle.size());
		for (size_t i=0; i<source_particle.size(); ++i)
		{
			RealVectorX unit_vector_in_ith_axis = zero_vector;
			unit_vector_in_ith_axis[i] = 1.0;

			RealVectorX shifted_right_source_particle = source_particle + epsilon * unit_vector_in_ith_axis;
			RealVectorX shifted_left_source_particle = source_particle - epsilon * unit_vector_in_ith_axis;

			gradient_W_over_x[i] = compute_M5_kernel(shifted_right_source_particle , destination_particle, neighbor_search_radius, number_of_dimension) - compute_M5_kernel(shifted_left_source_particle , destination_particle, neighbor_search_radius, number_of_dimension);

		}

		gradient_W_over_x = gradient_W_over_x / (2.0 * epsilon);
	}

	return gradient_W_over_x;
}

RealVectorX KernelHandler::gradient_of_M6_kernel( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, bool analytical_solution, Real epsilon )
{
	int m_number_of_dimension = number_of_dimension - 1;

	RealVectorX gradient_W_over_x(source_particle.size());

	Eigen::Vector3d sigma_list(1.0/120.0, 7.0/(478.0*(Real)M_PI), 1.0/(120.0*(Real)M_PI));
	Real sigma = sigma_list[m_number_of_dimension];

	if (analytical_solution)
	{
		Real h = neighbor_search_radius / 3.0;
		Real q = (source_particle - destination_particle).norm() / h;

		RealVectorX gradient_q_over_x = (source_particle-destination_particle) / (source_particle-destination_particle).norm() / h;

		if (q >= 2.0 && q < 3.0)
		{
			Real gradient_w_over_q = sigma * (-5.0) * std::pow(3.0-q, 4.0);
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
		else if (q >= 1.0 && q < 2.0)
		{
			Real gradient_w_over_q = sigma * ( (-5.0) * std::pow(3.0-q, 4.0) + 30.0 * std::pow(2.0-q, 4.0) );
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
		else if (q >= 0.0 && q < 1.0)
		{
			Real gradient_w_over_q = sigma * ( (-5.0) * std::pow(3.0-q, 4.0) + 30.0 * std::pow(2.0-q, 4.0) - 75.0 * std::pow(1.0-q, 4.0) );
			gradient_W_over_x = gradient_w_over_q * 1.0/std::pow(h, 3.0) * gradient_q_over_x;
		}
	} else {
		RealVectorX zero_vector = RealVectorX::Zero(source_particle.size());
		for (size_t i=0; i<source_particle.size(); ++i)
		{
			RealVectorX unit_vector_in_ith_axis = zero_vector;
			unit_vector_in_ith_axis[i] = 1.0;

			RealVectorX shifted_right_source_particle = source_particle + epsilon * unit_vector_in_ith_axis;
			RealVectorX shifted_left_source_particle = source_particle - epsilon * unit_vector_in_ith_axis;

			gradient_W_over_x[i] = compute_M6_kernel(shifted_right_source_particle , destination_particle, neighbor_search_radius, number_of_dimension) - compute_M6_kernel(shifted_left_source_particle , destination_particle, neighbor_search_radius, number_of_dimension);
		}

		gradient_W_over_x = gradient_W_over_x / (2.0 * epsilon);
	}

	return gradient_W_over_x;
}

Real KernelHandler::test_M4_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, Real epsilon )
{
	RealVectorX analytical_solution_of_gradient = gradient_of_M4_kernel(source_particle, destination_particle, neighbor_search_radius, number_of_dimension, true);
	RealVectorX approximate_solution_of_gradient = gradient_of_M4_kernel(source_particle, destination_particle, neighbor_search_radius, number_of_dimension, false, epsilon);

	Real error_rate = (analytical_solution_of_gradient - approximate_solution_of_gradient).norm() / analytical_solution_of_gradient.norm();
	return error_rate;
}

Real KernelHandler::test_M5_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, Real epsilon )
{
	RealVectorX analytical_solution_of_gradient = gradient_of_M5_kernel(source_particle, destination_particle, neighbor_search_radius, number_of_dimension, true);
	RealVectorX approximate_solution_of_gradient = gradient_of_M5_kernel(source_particle, destination_particle, neighbor_search_radius, number_of_dimension, false, epsilon);

	Real error_rate = (analytical_solution_of_gradient - approximate_solution_of_gradient).norm() / analytical_solution_of_gradient.norm();
	return error_rate;
}

Real KernelHandler::test_M6_gradient( Eigen::Ref<RealVectorX> source_particle, Eigen::Ref<RealVectorX> destination_particle, Real neighbor_search_radius, int number_of_dimension, Real epsilon )
{
	RealVectorX analytical_solution_of_gradient = gradient_of_M6_kernel(source_particle, destination_particle, neighbor_search_radius, number_of_dimension, true);
	RealVectorX approximate_solution_of_gradient = gradient_of_M6_kernel(source_particle, destination_particle, neighbor_search_radius, number_of_dimension, false, epsilon);

	Real error_rate = (analytical_solution_of_gradient - approximate_solution_of_gradient).norm() / analytical_solution_of_gradient.norm();
	return error_rate;
}

Real KernelHandler::integrate_M4_kernel(Real h, int number_of_dimension)
{
	int m_number_of_dimension = number_of_dimension - 1;

	RealVectorX origin = RealVectorX::Zero(number_of_dimension);

	Real integration = 0.0;

	Real step_size = h / 50.0;

	if (number_of_dimension == 3)
	{
		RealVectorX dest = RealVectorX::Zero(number_of_dimension);
		for (Real i = -2*h; i <= 2*h; i += step_size)
		{
			for (Real j = -2*h; j <= 2*h; j += step_size)
			{
				for (Real k = -2*h; k <= 2*h; k += step_size)
				{
					dest[0] = i;
					dest[1] = j;
					dest[2] = k;

					integration += compute_M4_kernel( origin, dest, h*2.0, number_of_dimension ) * step_size * step_size * step_size;
				}
			}
		}
	} else if (number_of_dimension == 2)
	{
		RealVectorX dest = RealVectorX::Zero(number_of_dimension);
		for (Real i = -2*h; i <= 2*h; i += step_size)
		{
			for (Real j = -2*h; j <= 2*h; j += step_size)
			{
				dest[0] = i;
				dest[1] = j;

				integration += compute_M4_kernel( origin, dest, h*2.0, number_of_dimension ) * step_size * step_size;
			}
		}
	} else if (number_of_dimension == 1)
	{
		RealVectorX dest = RealVectorX::Zero(number_of_dimension);
		for (Real i = -2*h; i <= 2*h; i += step_size)
		{
			dest[0] = i;
			integration += compute_M4_kernel( origin, dest, h*2.0, number_of_dimension ) * step_size;
		}
	} else {
		return -1;
	}

	return integration;
}
//float KernelHandler::integrate_M5_kernel()
//float KernelHandler::integrate_M6_kernel()

