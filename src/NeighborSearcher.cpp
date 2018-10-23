#include "NeighborSearcher.hpp"

#include "math_types.hpp"

#include <merely3d/merely3d.hpp>
#include <CompactNSearch/CompactNSearch>

#include <algorithm>    // std::sort
#include <numeric>

#include <iostream>


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

NeighborSearcher::NeighborSearcher()
{

}

std::vector<size_t> NeighborSearcher::find_neighbors_within_radius(std::vector<RealVector3> &particles, size_t selected_particle_index, double radius, bool use_compactN)
{
	std::vector<size_t> m_neighbors;
	if (use_compactN)
		m_neighbors = compactN_neighbor_search(particles, selected_particle_index, radius);
	else
		m_neighbors = brute_force_neighbor_search(particles, selected_particle_index, radius);

	return m_neighbors;
}

std::vector<std::array<CompactNSearch::Real, 3>> NeighborSearcher::convect_to_CompactN_position(std::vector<RealVector3> &particles)
{
	std::vector<std::array<CompactNSearch::Real, 3>> CompactN_particles;
	for (size_t i=0; i<particles.size(); ++i)
	{
		CompactN_particles.push_back(std::array<CompactNSearch::Real, 3>{ particles[i][0], particles[i][1], particles[i][2] });
	}
	return CompactN_particles;
}

std::vector<size_t> NeighborSearcher::compactN_neighbor_search(std::vector<RealVector3> &particles, size_t selected_particle_index, double radius)
{
	CompactNSearch::NeighborhoodSearch nsearch(radius);
	std::vector<std::array<CompactNSearch::Real, 3>> positions = convect_to_CompactN_position(particles);

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

std::vector<size_t> NeighborSearcher::brute_force_neighbor_search(std::vector<RealVector3> &particles, size_t selected_particle_index, double radius)
{
	// Brute Force
	std::vector<double> distances_to_selected_particle;
	for (size_t i=0; i<particles.size(); ++i)
	{
		RealVector3 distance_vector = particles[i] - particles[selected_particle_index];
		distances_to_selected_particle.push_back(distance_vector.squaredNorm());
	}

	// initialize original index locations
	std::vector<size_t> indices(distances_to_selected_particle.size());
	std::iota(indices.begin(), indices.end(), 0);

	// index sorting
	std::sort(indices.begin(), indices.end(),
		   [&distances_to_selected_particle](size_t i1, size_t i2) {return distances_to_selected_particle[i1] < distances_to_selected_particle[i2];});

	size_t boundary = 0;
	for (size_t i=0; i<particles.size(); ++i)
	{
		if (distances_to_selected_particle[indices[i]] > (radius*radius))
		{
			boundary = i;
			break;
		}
	}
	std::vector<size_t> m_neighbors(indices.begin(), indices.begin()+boundary);
	return m_neighbors;
}

void NeighborSearcher::brute_force_search(std::vector<RealVector3> &particles, std::vector<std::vector<int> >& n_neighbors_indices, double radius)
{
    int k(particles.size());
    n_neighbors_indices.reserve(k);

    RealVector3 vec1,vec2;

    for(int i = 0; i < k; i++ )
    {
        std::vector<int> neighbors_indices;
        for(int j = 0; j < k; j++)
        {
        	vec1 = particles[i];
            if(j != i)
            {
                vec2 = particles[j];
                RealVector3 diff_vec = vec2 - vec1;
                if(diff_vec.dot(diff_vec) <= radius*radius)
                    neighbors_indices.push_back(j);
            }
        }
        n_neighbors_indices.push_back(neighbors_indices);
    }
}

