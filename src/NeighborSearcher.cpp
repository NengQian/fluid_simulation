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

void NeighborSearcher::set_boundary_particles_ptr(std::vector<RealVector3>& boundary_particles)
{
	boundary_particles_ptr = std::make_shared<std::vector<RealVector3>>(boundary_particles);
}

// already set inactive
std::vector< std::vector<size_t> > NeighborSearcher::find_neighbors_in_boundary( )
{
	CompactNSearch::NeighborhoodSearch nsearch(neighbor_search_radius);
	std::vector<std::array<CompactNSearch::Real, 3>> boundary_positions = convect_to_CompactN_position(*boundary_particles_ptr);
	std::vector<std::array<CompactNSearch::Real, 3>> particle_positions = convect_to_CompactN_position(*particles_ptr);

	unsigned int point_set_id_1 = nsearch.add_point_set(particle_positions.front().data(), particle_positions.size());
	unsigned int point_set_id_2 = nsearch.add_point_set(boundary_positions.front().data(), boundary_positions.size());
	nsearch.set_active(point_set_id_2, point_set_id_1, false);
	nsearch.find_neighbors();

    CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id_1);
    std::vector< std::vector<size_t> > m_neighbors;
    m_neighbors.reserve(particle_positions.size());

	for (size_t i =0; i < particle_positions.size(); ++i)
	{
		std::vector<size_t> ns;
		// n_neighbors: returns Number of neighbors of point i in point set point_set
		for (size_t j = 0; j < ps.n_neighbors(point_set_id_2, i); ++j)
		{
			// Return PointID of the jth neighbor of the ith particle in the 2nd point set.
			unsigned int pid = ps.neighbor(point_set_id_2, i, j);
			ns.push_back(pid);
		}
		m_neighbors.push_back(ns);
	}

	return m_neighbors;
}

// neighbors include itself
std::vector< std::vector<size_t> > NeighborSearcher::find_boundary_neighbors( )
{
	CompactNSearch::NeighborhoodSearch nsearch(neighbor_search_radius);
	std::vector<std::array<CompactNSearch::Real, 3>> boundary_positions = convect_to_CompactN_position(*boundary_particles_ptr);

	// ... Fill array with 3 * n real numbers representing three-dimensional point positions.
	unsigned int point_set_id = nsearch.add_point_set(boundary_positions.front().data(), boundary_positions.size());
    nsearch.find_neighbors();

    CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id);
    std::vector< std::vector<size_t> > m_neighbors;

    for (size_t i = 0; i < ps.n_points(); ++i)
    {
    	std::vector<size_t> neighbors_of_i;
    	neighbors_of_i.push_back(i);
        for (size_t j = 0; j < ps.n_neighbors(point_set_id, i); ++j)
        {
        	neighbors_of_i.push_back(ps.neighbor(point_set_id, i, j));
        }
        m_neighbors.push_back(neighbors_of_i);
    }

	return m_neighbors;
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

std::vector< std::vector<size_t> > NeighborSearcher::find_neighbors_within_radius( std::vector<size_t> point_set )
{
	CompactNSearch::NeighborhoodSearch nsearch(neighbor_search_radius);
	std::vector<std::array<CompactNSearch::Real, 3>> positions = convect_to_CompactN_position();

	unsigned int point_set_id = nsearch.add_point_set(positions.front().data(), positions.size());
	nsearch.find_neighbors();

    CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id);
    std::vector< std::vector<size_t> > m_neighbors;
    m_neighbors.reserve(point_set.size());

	for (size_t i =0; i < point_set.size(); ++i)
	{
		std::vector<size_t> ns;
		for (size_t j = 0; j < ps.n_neighbors(point_set_id, point_set[i]); ++j)
		{
			// Return PointID of the jth neighbor of the ith particle in the 0th point set.
			unsigned int pid = ps.neighbor(point_set_id, point_set[i], j);
			ns.push_back(pid);
		}
		m_neighbors.push_back(ns);
	}

	return m_neighbors;
}

std::vector< std::vector<size_t> > NeighborSearcher::find_neighbors_within_radius( bool use_compactN )
{
	if (use_compactN)
		return compactN_neighbor_search( );
	else
		return brute_force_neighbor_search( );

	return {};
}

std::vector< std::vector<size_t> > NeighborSearcher::find_neighbors_within_radius( std::vector<RealVector3>& point_set )
{
	CompactNSearch::NeighborhoodSearch nsearch(neighbor_search_radius);
	std::vector<std::array<CompactNSearch::Real, 3>> sample_positions = convect_to_CompactN_position(point_set);
	std::vector<std::array<CompactNSearch::Real, 3>> particle_positions = convect_to_CompactN_position();

	unsigned int point_set_id_1 = nsearch.add_point_set(particle_positions.front().data(), particle_positions.size());
	unsigned int point_set_id_2 = nsearch.add_point_set(sample_positions.front().data(), sample_positions.size());
	nsearch.set_active(point_set_id_1, point_set_id_2, false);
	nsearch.find_neighbors();

    CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id_2);
    std::vector< std::vector<size_t> > m_neighbors;
    m_neighbors.reserve(point_set.size());

	for (size_t i =0; i < point_set.size(); ++i)
	{
		std::vector<size_t> ns;
		for (size_t j = 0; j < ps.n_neighbors(point_set_id_1, i); ++j)
		{
			// Return PointID of the jth neighbor of the ith particle in the 0th point set.
			unsigned int pid = ps.neighbor(point_set_id_1, i, j);
			ns.push_back(pid);
		}
		m_neighbors.push_back(ns);
	}

	return m_neighbors;
}

std::vector<std::array<CompactNSearch::Real, 3>> NeighborSearcher::convect_to_CompactN_position( std::vector<RealVector3>& point_set )
{
	std::vector<std::array<CompactNSearch::Real, 3>> CompactN_particles;
	for (size_t i=0; i<point_set.size(); ++i)
	{
		CompactN_particles.push_back(std::array<CompactNSearch::Real, 3>{ point_set[i][0], point_set[i][1], point_set[i][2] });
	}
	return CompactN_particles;
}

std::vector<std::array<CompactNSearch::Real, 3>> NeighborSearcher::convect_to_CompactN_position( )
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

	unsigned int point_set_id = nsearch.add_point_set(positions.front().data(), positions.size());
	nsearch.find_neighbors();

    CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id);
	std::vector<size_t> m_neighbors;

	for (size_t i = 0; i < ps.n_neighbors(point_set_id, selected_particle_index); ++i)
	{
		// Return PointID of the jth neighbor of the ith particle in the 0th point set.
		unsigned int pid = ps.neighbor(point_set_id, selected_particle_index, i);
		m_neighbors.push_back(pid);
	}
	return m_neighbors;
}

// neighbors include itself
std::vector< std::vector<size_t> > NeighborSearcher::compactN_neighbor_search( )
{
	CompactNSearch::NeighborhoodSearch nsearch(neighbor_search_radius);
	std::vector<std::array<CompactNSearch::Real, 3>> positions = convect_to_CompactN_position();

	// ... Fill array with 3 * n real numbers representing three-dimensional point positions.
	unsigned int point_set_id = nsearch.add_point_set(positions.front().data(), positions.size());
    nsearch.find_neighbors();

    CompactNSearch::PointSet const& ps = nsearch.point_set(point_set_id);
    std::vector< std::vector<size_t> > m_neighbors;

    for (size_t i = 0; i < ps.n_points(); ++i)
    {
    	std::vector<size_t> neighbors_of_i;
    	neighbors_of_i.push_back(i);
        for (size_t j = 0; j < ps.n_neighbors(point_set_id, i); ++j)
        {
        	neighbors_of_i.push_back(ps.neighbor(point_set_id, i, j));
        }
        m_neighbors.push_back(neighbors_of_i);
    }

	return m_neighbors;
}

std::vector<size_t> NeighborSearcher::brute_force_neighbor_search( size_t selected_particle_index )
{
    size_t k = particles_ptr->size();
    size_t i = selected_particle_index;

    std::vector<size_t> neighbors_of_i;

    RealVector3 vec1,vec2;

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

    return neighbors_of_i;
}

std::vector< std::vector<size_t> > NeighborSearcher::brute_force_neighbor_search( )
{
    size_t k = particles_ptr->size();
	std::vector< std::vector<size_t> > m_neighbors;
	m_neighbors.reserve(k);

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
        m_neighbors.push_back(neighbors_of_i);
    }

    return m_neighbors;
}

