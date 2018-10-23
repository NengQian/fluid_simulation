#include <catch.hpp>

#include "KernelHandler.hpp"
#include "math_types.hpp"

#include <cmath>

using namespace Simulator;

const Real error = std::pow(10.0, -6.0);

TEST_CASE( "M4 Kernels are computed", "[M4 Kernel]" ) {

	SECTION( "when q is 0" ) {
		RealVector3 s(0.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M4_kernel(s, d, 2.0, 3) - 1.0/(Real)M_PI) <= error );
	}

	SECTION( "when q is 1" ) {
		RealVector3 s(1.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M4_kernel(s, d, 2.0, 3) - (1.0/(4.0*(Real)M_PI))) <= error );
	}

	SECTION( "when q is 2" ) {
		RealVector3 s(2.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M4_kernel(s, d, 2.0, 3) - 0.0) <= error );
	}
}

TEST_CASE( "M5 Kernels are computed", "[M5 Kernel]" ) {

	SECTION( "when q is 0" ) {
		RealVector3 s(0.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M5_kernel(s, d, 2.5, 3) - 230.0 / (16.0 * 20.0 * (Real)M_PI)) <= error );
	}

	SECTION( "when q is 0.5" ) {
		RealVector3 s(0.5, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M5_kernel(s, d, 2.5, 3) - 11.0 / (20.0 * (Real)M_PI)) <= error);
	}

	SECTION( "when q is 1.5" ) {
		RealVector3 s(1.5, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M5_kernel(s, d, 2.5, 3) - (1.0/(20.0*(Real)M_PI))) <= error);
	}

	SECTION( "when q is 2.5" ) {
		RealVector3 s(2.5, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M5_kernel(s, d, 2.5, 3) - 0.0) <= error );
	}
}

TEST_CASE( "M6 Kernels are computed", "[M6 Kernel]" ) {

	SECTION( "when q is 0" ) {
		RealVector3 s(0.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M6_kernel(s, d, 3.0, 3) - 66.0 / (120.0 * (Real)M_PI)) <= error );
	}

	SECTION( "when q is 1" ) {
		RealVector3 s(1.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M6_kernel(s, d, 3.0, 3) - 26.0 / (120.0 * (Real)M_PI)) <= error );
	}

	SECTION( "when q is 2" ) {
		RealVector3 s(2.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M6_kernel(s, d, 3.0, 3) - (1.0/(120.0*(Real)M_PI))) <= error);
	}

	SECTION( "when q is 3" ) {
		RealVector3 s(3.0, 0.0, 0.0);
		RealVector3 d(0.0, 0.0, 0.0);

		KernelHandler kh;
	    REQUIRE( std::abs(kh.compute_M6_kernel(s, d, 3.0, 3) - 0.0) <= error );
	}
}

TEST_CASE( "M4 Kernels is integrated", "[M4 Kernel Integration]" ) {

	SECTION( "h == 1, dim == 1" ) {
		KernelHandler kh;
	    REQUIRE( std::abs(kh.integrate_M4_kernel(1.0, 1) - 1.0) <= error );
	}

	SECTION( "h == 1, dim == 2" ) {
		KernelHandler kh;
	    REQUIRE( std::abs(kh.integrate_M4_kernel(1.0, 2) - 1.0) <= error );
	}

	SECTION( "h == 1, dim == 3" ) {
		KernelHandler kh;
	    REQUIRE( std::abs(kh.integrate_M4_kernel(1.0, 3) - 1.0) <= error );
	}
}

