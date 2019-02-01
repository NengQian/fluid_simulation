#include <catch.hpp>

#include "KernelHandler.hpp"
#include "math_types.hpp"

#include <cmath>

using namespace Simulator;

const Real error = std::pow(10.0, -6.0);

TEST_CASE( "M4 Kernels are computed", "[M4 Kernel]" ) {

	KernelHandler kh(2.0);

	SECTION( "when q is 0" ) {
		RealVector3 s(0.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 4) - 1.0/(Real)M_PI) <= error );
	}

	SECTION( "when q is 1" ) {
		RealVector3 s(1.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 4) - (1.0/(4.0*(Real)M_PI))) <= error );
	}

	SECTION( "when q is 2" ) {
		RealVector3 s(2.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 4) - 0.0) <= error );
	}
}

TEST_CASE( "M5 Kernels are computed", "[M5 Kernel]" ) {

	KernelHandler kh(2.5);

	SECTION( "when q is 0" ) {
		RealVector3 s(0.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 5) - 230.0 / (16.0 * 20.0 * (Real)M_PI)) <= error );
	}

	SECTION( "when q is 0.5" ) {
		RealVector3 s(0.5, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 5) - 11.0 / (20.0 * (Real)M_PI)) <= error);
	}

	SECTION( "when q is 1.5" ) {
		RealVector3 s(1.5, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 5) - (1.0/(20.0*(Real)M_PI))) <= error);
	}

	SECTION( "when q is 2.5" ) {
		RealVector3 s(2.5, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 5) - 0.0) <= error );
	}
}

TEST_CASE( "M6 Kernels are computed", "[M6 Kernel]" ) {

	KernelHandler kh(3.0);

	SECTION( "when q is 0" ) {
		RealVector3 s(0.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 6) - 66.0 / (120.0 * (Real)M_PI)) <= error );
	}

	SECTION( "when q is 1" ) {
		RealVector3 s(1.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 6) - 26.0 / (120.0 * (Real)M_PI)) <= error );
	}

	SECTION( "when q is 2" ) {
		RealVector3 s(2.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 6) - (1.0/(120.0*(Real)M_PI))) <= error);
	}

	SECTION( "when q is 3" ) {
		RealVector3 s(3.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

	    REQUIRE( std::abs(kh.compute_kernel(s, d, 6) - 0.0) <= error );
	}
}

TEST_CASE( "M4 Kernels is integrated", "[M4 Kernel Integration]" ) {

	KernelHandler kh(5.0);

	SECTION( "dim == 1" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 4, 1 ) - 1.0) <= error );
	}

	SECTION( "dim == 2" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 4, 2 ) - 1.0) <= error );
	}

	SECTION( "dim == 3" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 4, 3 ) - 1.0) <= error );
	}
}

TEST_CASE( "M5 Kernels is integrated", "[M5 Kernel Integration]" ) {

	KernelHandler kh(5.0);

	SECTION( "dim == 1" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 5, 1 ) - 1.0) <= error );
	}

	SECTION( "dim == 2" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 5, 2 ) - 1.0) <= error );
	}

	SECTION( "dim == 3" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 5, 3 ) - 1.0) <= error );
	}
}

TEST_CASE( "M6 Kernels is integrated", "[M6 Kernel Integration]" ) {

	KernelHandler kh(5.0);

	SECTION( "dim == 1" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 6, 1 ) - 1.0) <= error );
	}

	SECTION( "dim == 2" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 6, 2 ) - 1.0) <= error );
	}

	SECTION( "dim == 3" ) {
	    REQUIRE( std::abs(kh.integrate_kernel( 6, 3 ) - 1.0) <= error );
	}
}

TEST_CASE( "Gradient of M4 Kernels are computed", "[M4 Kernel Gradient]" ) {

	KernelHandler kh(2.0);
	int N = 100;

	for (int i=0; i<N; ++i)
	{
		double x = static_cast<double>(2.0*i/N+1.0/N);
		SECTION( "when q is " + std::to_string(x) ) {
			RealVector3 s(x, 0.0, 0.0);
			RealVector3 d(0.0, 0.0, 0.0);

	    	REQUIRE( std::abs(kh.test_gradient(s, d, 4)) <= error );
		}
	}
}

TEST_CASE( "Gradient of M5 Kernels are computed", "[M5 Kernel Gradient]" ) {

	KernelHandler kh(2.5);

	int N = 100;

	for (int i=0; i<N; ++i)
	{
		double x = static_cast<double>(2.5*i/N+2.5/2.0/N);
		SECTION( "when q is " + std::to_string(x) ) {
			RealVector3 s(x, 0.0, 0.0);
			RealVector3 d(0.0, 0.0, 0.0);

	    	REQUIRE( std::abs(kh.test_gradient(s, d, 5)) <= error );
		}
	}
}

TEST_CASE( "Gradient of M6 Kernels are computed", "[M6 Kernel Gradient]" ) {

	KernelHandler kh(3.0);

	int N = 100;

	for (int i=0; i<N; ++i)
	{
		double x = static_cast<double>(3.0*i/N+3.0/2.0/N);
		SECTION( "when q is " + std::to_string(x) ) {
			RealVector3 s(x, 0.0, 0.0);
			RealVector3 d(0.0, 0.0, 0.0);

	    	REQUIRE( std::abs(kh.test_gradient(s, d, 6)) <= error );
		}
	}
}