#ifndef SPHSIMULATOR_RIGID_BODY_H
#define SPHSIMULATOR_RIGID_BODY_H
#include "SPHSimulator.hpp"

#include <cstdlib>

#define WCSPH 0
#define PBFSPH 1

class SPHSimulator_rigid_body : public SPHSimulator
{
public:
    SPHSimulator_rigid_body(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH, int solver_type)
	 : SPHSimulator(N, uParticle_len,dt, eta, B, alpha, rest_density, solver_type)
	{
        if (with_viscosity == 1)
        	viscosity_flag = true;
        else
        	viscosity_flag = false;

        if (with_XSPH == 1)
        	XSPH_flag = true;
        else
        	XSPH_flag = false;
    }

    virtual void update_simulation() override
    {
    	switch(solver_type)
    	{
    		case WCSPH:
    			update_simulation_WCSPH();
    			break;
    		case PBFSPH:
    			update_simulation_PBFSPH();
    			break;
    		default:
    			std::cout << "Error: unknown solver type" << std::endl;
    	        std::exit(-1);
    	}
    }

    //virtual void generate_particles() = 0;

protected:
    // should I also move all boundary related function to here? Or maybe not...
	bool viscosity_flag = true;
	bool XSPH_flag = true;

	//int solver_type;
	int epoch = 5;

	void update_simulation_WCSPH()
	{
        std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
        std::vector< std::vector<size_t> > neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

        Real r = neighbor_search_radius;
        particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, r);

        std::vector<RealVector3> external_forces;
        for (size_t i=0; i<particles.size(); ++i)
            external_forces.push_back( gravity * particles[i].mass ); //Neng: we have the gravity in class private

        std::vector<RealVector3> as = particleFunc.update_acceleration( particles, boundary_particles, neighbors_set, neighbors_in_boundary, external_forces, r, viscosity_flag);
        particleFunc.update_velocity(particles, dt, as);

        if (XSPH_flag == false)
        {
            particleFunc.update_position(particles, dt);
        } else {
            /* --------- using XSPH -------------------*/
            particleFunc.update_position(particles, dt, neighbors_set, r);
        }

        update_positions();
	}

	void update_simulation_PBFSPH()
	{
		std::vector< std::vector<size_t> > neighbors_set;
		std::vector< std::vector<size_t> > neighbors_in_boundary;
        Real r = neighbor_search_radius;

		// Step 0: save the current position information before any updates
		std::vector<RealVector3> old_positions;
		for (auto& pos : positions)
		{
			old_positions.push_back(RealVector3(pos));
        	//std::cout << pos[2] << " " << std::endl;
		}

		// Step 1: preview of particles's status
    	for (size_t i=0; i<particles.size(); ++i)
		{
    		particles[i].velocity += gravity * dt;
		}

    	//particleFunc.update_position(particles, dt);

    	if (XSPH_flag == false)
    	{
    		particleFunc.update_position(particles, dt);
    	} else { // use XSPH
        	neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
        	neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

        	particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, r);

            particleFunc.update_position(particles, dt, neighbors_set, r);
    	}

    	update_positions(); // needed

    	/*
		for (auto& pos : positions)
        	std::cout << pos[2] << std::endl;
		*/

        // Step 2: search neighbors
        neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
        neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

        // Step 3: iteration of lambda and position computing
        for (int itr=0; itr<epoch; ++itr)
        {
        	// Step 3.0: compute density
            particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, r);

            /*
            std::cout << "in " << itr << "-th iteration" << std::endl;
            for (auto& d : densities)
            	std::cout << d << " ";

			std::cout << std::endl;
			*/

        	// Step 3.1: compute lambda
        	std::vector<Real> lambda; // only contain lambda of fluid particles, cuz lambda
        	lambda.clear();

        	for (size_t i=0; i<particles.size(); ++i)
        	{
        		mParticle P_i = particles[i];

        		size_t number_of_fluid_neighbors_of_i = neighbors_set[i].size();
        		size_t number_of_boundary_neighbors_of_i = neighbors_in_boundary[i].size();

        		Real C_i = P_i.density / rest_density - 1;
        		std::vector<RealVector3> grad_of_C_i; // vector with size #fluid_neighbors + #boundary_neighbors
        		grad_of_C_i.clear();

        		Real S_i = 0.0;

        		// compute grad_of_C_i and S_i
        		for (size_t k=0; k<(number_of_fluid_neighbors_of_i+number_of_boundary_neighbors_of_i); ++k)
        		{
        			size_t j;
        			mParticle NP_ij;

        			if (k == 0) // that is, j == i
        			{
        				// skip first, will later compute it by adding up all other gradients
        				grad_of_C_i.push_back(RealVector3(0.0, 0.0, 0.0));
        				continue;
        			}
        			else if (k < number_of_fluid_neighbors_of_i) // j != i and j is fluid neighbor
        			{
            			j = neighbors_set[i][k];
            			NP_ij = particles[j];
           			}
        			else { // j != i and j is boundary neighbor
           				j = neighbors_in_boundary[i][k-number_of_fluid_neighbors_of_i];
           				NP_ij = boundary_particles[j];
            		}

					RealVector3 grad_W = kernelHandler.gradient_of_kernel( P_i.position, NP_ij.position, 4 );
					RealVector3 grad_Cij = (-NP_ij.mass / rest_density) * grad_W;

					grad_of_C_i.push_back(grad_Cij);

					// subtract it with grad_Cii
					grad_of_C_i[0] -= grad_Cij;

					// update S_i, but here got to skip C_ii
					S_i += (grad_Cij[0]*grad_Cij[0] + grad_Cij[1]*grad_Cij[1] + grad_Cij[2]*grad_Cij[2]) / NP_ij.mass;
        		}

        		// add C_ii to complete S_i
        		RealVector3 grad_Cii = grad_of_C_i[0];
        		S_i += (grad_Cii[0]*grad_Cii[0] + grad_Cii[1]*grad_Cii[1] + grad_Cii[2]*grad_Cii[2]) / P_i.mass;

        		// compute lambda_i and push back
        		Real lambda_i = 0.0;

        		if (C_i > 0.0)
        		{
        			lambda_i = -C_i / (S_i + 0.0001);
        		}

        		lambda.push_back(lambda_i);
        	}

        	// Step 3.2: correct position
        	std::vector<RealVector3> dx;
        	for (size_t i=0; i<particles.size(); ++i)
        	{
        		Real lambda_i = lambda[i];
        		mParticle P_i = particles[i];
        		RealVector3 dx_i = RealVector3(0.0, 0.0, 0.0);

        		size_t number_of_fluid_neighbors_of_i = neighbors_set[i].size();
        		size_t number_of_boundary_neighbors_of_i = neighbors_in_boundary[i].size();

        		// sum up the contribution of all fluid and boundary neighbors
        		for (size_t k=0; k<(number_of_fluid_neighbors_of_i+number_of_boundary_neighbors_of_i); ++k)
        		{
        			size_t j;
        			mParticle NP_ij;
        			Real lambda_j;

        			if (k == 0) // that is, j == i
        			{
        				NP_ij = P_i;
        				lambda_j = lambda_i;
        			}
        			else if (k < number_of_fluid_neighbors_of_i) // j != i and j is fluid neighbor
        			{
            			j = neighbors_set[i][k];
            			NP_ij = particles[j];
            			lambda_j = lambda[j];
           			}
        			else { // j != i and j is boundary neighbor
           				j = neighbors_in_boundary[i][k-number_of_fluid_neighbors_of_i];
           				NP_ij = boundary_particles[j];
        				lambda_j = lambda_i; // <----- idea from the assignment sheet, not quite understand though
            		}

					RealVector3 grad_W_ij = kernelHandler.gradient_of_kernel( P_i.position, NP_ij.position, 4 );

        			dx_i += (lambda_i * NP_ij.mass / P_i.mass + lambda_j) * grad_W_ij;
        		}

        		// computing of dx_i completed here
        		dx_i /= rest_density;

        		dx.push_back(dx_i);
        	}

        	// Step 4: update position after every iteration <---- have to do it separately
        	for (size_t i=0; i<particles.size(); ++i)
        	{
        		particles[i].position += dx[i];
        		//update_positions();
        	}
        }

        /*
		for (auto& pos : positions)
        	std::cout << pos[2] << std::endl;
        */

        // Step 5: update velocity after the epochs
    	for (size_t i=0; i<particles.size(); ++i)
    	{
    		particles[i].velocity = (particles[i].position - old_positions[i]) / dt;
    	}

    	/*
    	// Experiment: Add XSPH at the final stage according to the paper
    	// (Optional)Step 6: add XSPH
        if (XSPH_flag == true)
        {
        	// need to recompute neighbors and densities
        	update_positions();

        	neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
        	neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

        	densities = particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, r);

            particleFunc.update_position(particles, dt, neighbors_set, r, densities);
        }
        */

        update_positions();
	}
};

#endif // SPHSIMULATOR_RIGID_BODY_H
