#include "NeighborSearcher.hpp"
#include "math_types.hpp"

#include <merely3d/merely3d.hpp>
#include <CompactNSearch/CompactNSearch>

#include <algorithm>    // std::sort
#include <numeric>
#include <iostream>

using namespace Simulator;

NeighborSearcher::NeighborSearcher()
{

}

NeighborSearcher::NeighborSearcher(Real radius)
{
	set_neighbor_search_radius(radius);
}


void NeighborSearcher::set_neighbor_search_radius(Real radius)
{
	neighbor_search_radius = radius;
}

void NeighborSearcher::set_particles_ptr(std::vector<RealVector3>& particles)
{
	particles_ptr = std::make_shared<std::vector<RealVector3>>(particles);
}

std::vector<size_t> NeighborSearcher::find_neighbors_within_radius( size_t selected_particle_index, bool use_compactN )
{
	std::vector<size_t> m_neighbors;
	if (use_compactN)
		m_neighbors = compactN_neighbor_search( selected_particle_index );
	else
		m_neighbors = brute_force_neighbor_search( selected_particle_index );

	return m_neighbors;
}

std::vector<std::array<CompactNSearch::Real, 3>> NeighborSearcher::convect_to_CompactN_position()
{
	std::vector<std::array<CompactNSearch::Real, 3>> CompactN_particles;
	for (size_t i=0; i<particles_ptr->size(); ++i)
	{
		CompactN_particles.push_back(std::array<CompactNSearch::Real, 3>{ (*particles_ptr)[i][0], (*particles_ptr)[i][1], (*particles_ptr)[i][2] });
	}
	return CompactN_particles;
}

std::vector<size_t> NeighborSearcher::compactN_neighbor_search( size_t selected_particle_index )
{
	CompactNSearch::NeighborhoodSearch nsearch(neighbor_search_radius);
	std::vector<std::array<CompactNSearch::Real, 3>> positions = convect_to_CompactN_position();

	// ... Fill array with 3 * n real numbers representing three-dimensional point positions.
	unsigned int point_set_id = nsearch.add_point_set(positions.front().data(), positions.size());
    nsearch.find_neighbors();

    CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id);
	std::vector<size_t> m_neighbors;

	m_neighbors.push_back(selected_particle_index);
	for (size_t i = 0; i < ps.n_neighbors(point_set_id, selected_particle_index); ++i)
	{
		// Return PointID of the jth neighbor of the ith particle in the 0th point set.
		unsigned int pid = ps.neighbor(0, selected_particle_index, i);
		m_neighbors.push_back(pid);
	}
	return m_neighbors;
}

std::vector<size_t> NeighborSearcher::brute_force_neighbor_search( size_t selected_particle_index )
{
	// Brute Force
	std::vector<Real> distances_to_selected_particle;
	for (size_t i=0; i<particles_ptr->size(); ++i)
	{
		RealVector3 distance_vector = (*particles_ptr)[i] - (*particles_ptr)[selected_particle_index];
		distances_to_selected_particle.push_back(distance_vector.squaredNorm());
	}

	// initialize original index locations
	std::vector<size_t> indices(distances_to_selected_particle.size());
	std::iota(indices.begin(), indices.end(), 0);

	// index sorting
	std::sort(indices.begin(), indices.end(),
		   [&distances_to_selected_particle](size_t i1, size_t i2) {return distances_to_selected_particle[i1] < distances_to_selected_particle[i2];});

	size_t boundary = 0;
	for (size_t i=0; i<particles_ptr->size(); ++i)
	{
		if (distances_to_selected_particle[indices[i]] > (neighbor_search_radius*neighbor_search_radius))
		{
			boundary = i;
			break;
		}
	}
	std::vector<size_t> m_neighbors(indices.begin(), indices.begin()+boundary);
	return m_neighbors;
}

std::vector< std::vector<size_t> > NeighborSearcher::brute_force_neighbor_search( )
{
    size_t k = particles_ptr->size();
	std::vector< std::vector<size_t> > neighbors;
	neighbors.reserve(k);

    RealVector3 vec1,vec2;

    for(size_t i = 0; i < k; i++ )
    {
        std::vector<size_t> neighbors_of_i;
        for(size_t j = 0; j < k; j++)
        {
        	vec1 = (*particles_ptr)[i];
            if(j != i)
            {
                vec2 = (*particles_ptr)[j];
                RealVector3 diff_vec = vec2 - vec1;
                if(diff_vec.dot(diff_vec) <= neighbor_search_radius*neighbor_search_radius)
                	neighbors_of_i.push_back(j);
            }
        }
        neighbors.push_back(neighbors_of_i);
    }

    return neighbors;
}

