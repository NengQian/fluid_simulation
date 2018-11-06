#include "ParticleFunc.hpp"

std::vector<Real> ParticleFunc::update_density(std::vector<std::vector<Particle>>& neighbors_of_set, std::vector<Particle>& particles )
{
	std::vector<Real> densities;
	for (size_t i=0; i<neighbors_of_set.size(); ++i)
	{
		Real d = 0.0;
		for (size_t j=0; j<neighbors_of_set[i].size(); ++j)
		{
			Real m = neighbors_of_set[i][j]->mass;
			d += m * kernelHandler.compute_kernel( particles[i]->position, neighbors_of_set[i][j]->position, 4 );
		}
	}
	return densities;
}

