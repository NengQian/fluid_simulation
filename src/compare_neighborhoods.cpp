/*
#include <memory>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <fstream>

#include <merely3d/app.hpp>
#include <merely3d/window.hpp>
#include <merely3d/camera_controller.hpp>

#include <Eigen/Geometry>

#include <imgui/imgui.h>
#include <imgui/imgui_event_handler.h>

#include <chrono>

#include "math_types.hpp"
#include "simulation.hpp"
#include "SPHSimulator.hpp"
#include <CompactNSearch/CompactNSearch>

using merely3d::Window;
using merely3d::WindowBuilder;
using merely3d::Frame;
using merely3d::Key;
using merely3d::Action;
using merely3d::EventHandler;
using merely3d::Material;
using merely3d::Color;
using merely3d::CameraController;
using merely3d::renderable;
using merely3d::Rectangle;
using merely3d::Box;
using merely3d::red;
using merely3d::Line;
using merely3d::Sphere;
using merely3d::Camera;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::AngleAxisf;

using Simulator::Real;

using namespace std::chrono;

int main()
{
    using Simulator::Simulation;
    using Simulator::Real;

    float m_radius = 5.0f;

    SPHSimulator sphSimulator(0.01);
    sphSimulator.generate_particles();

	//Real radius = static_cast<Real>(sphSimulator.get_neighbor_search_radius);
	high_resolution_clock::time_point t1, t2;

    {
		std::vector<std::vector<size_t> > n_neighbors_indices;

		high_resolution_clock::time_point t1 = high_resolution_clock::now();
		n_neighbors_indices = sphSimulator.find_neighbors_of_all( false );
		high_resolution_clock::time_point t2 = high_resolution_clock::now();

		auto dura = duration_cast<microseconds>( t2 - t1 ).count();

		std::cout<< "the neighbor indices is:"<< std::endl;
		for(size_t i=0;i<n_neighbors_indices.size();i++)
		{
			std::cout<< " "<< i;
			for(size_t j=0; j<n_neighbors_indices[i].size();j++)
				std::cout<<" - "<< n_neighbors_indices[i][j];
			std::cout<<std::endl;
		}
		std::cout<< "Brute_force_search duration "<< dura << std::endl;
    }

    {
		std::vector<std::vector<size_t> > n_neighbors_indices;

		t1 = high_resolution_clock::now();
		n_neighbors_indices = sphSimulator.find_neighbors_of_all( true );
		t2 = high_resolution_clock::now();

        auto dura = duration_cast<microseconds>( t2 - t1 ).count();

        std::cout<< "the neighbor indices from CompactNSearch is:"<< std::endl;
		for(size_t i=0;i<n_neighbors_indices.size();i++)
		{
			std::cout<< " "<< i;
			for(size_t j=0; j<n_neighbors_indices[i].size();j++)
				std::cout<<" - "<< n_neighbors_indices[i][j];
			std::cout<<std::endl;
		}
        std::cout<< "compactNsearch duration "<< dura << std::endl;
    }

    // so now is try to run more case with different radius and number of particles
    std::vector<int> particles_num_vector({10,100,1000,10000});
    std::vector<double> radius_num_vector({0.1, 0.5, 1.0, 1.5});
    std::vector<std::vector<microseconds> > cns_runtimes_vector;
    std::vector<std::vector<microseconds> > bfs_runtimes_vector;

    for(int i=0;i<particles_num_vector.size();++i)
    {
    	SPHSimulator compare_nb(0.01);
        compare_nb.set_number_of_particles(static_cast<size_t>(particles_num_vector[i]));
        compare_nb.generate_particles();
        std::cout<<"particles number is "<< compare_nb.get_number_of_particles() <<std::endl;

        //std::cout<<"particles number is "<< particles_num_vector[i] <<std::endl;
        std::vector<microseconds> cns_runtime_vector;
        std::vector<microseconds> bfs_runtime_vector;
        for(int j=0;j<radius_num_vector.size();++j)
        {

            std::cout<<"radius is "<< radius_num_vector[j] <<std::endl;
            // try compactNSearch
            compare_nb.set_neighbor_search_radius(static_cast<float>(radius_num_vector[j]));

            t1 = high_resolution_clock::now();
            compare_nb.find_neighbors_of_all( true );
            t2 = high_resolution_clock::now();
            auto dura = duration_cast<microseconds>( t2 - t1 );
            cns_runtime_vector.push_back(dura);
            std::cout<<"cns duration:"<< dura.count() << std::endl;

            t1 = high_resolution_clock::now();
            compare_nb.find_neighbors_of_all( false );
            t2 = high_resolution_clock::now();
            dura = duration_cast<microseconds>( t2 - t1 );
            bfs_runtime_vector.push_back(dura);
            std::cout<<"bf duration:"<< dura.count() << std::endl;


        }
        std::cout<<""<<std::endl;
        cns_runtimes_vector.push_back(cns_runtime_vector);
        bfs_runtimes_vector.push_back(bfs_runtime_vector);

    }


    // now try to save the result to some file
    std::ofstream myfile;
    myfile.open ("compare_nbs.txt");
    for(int i=0;i<bfs_runtimes_vector.size();++i)
    {
        for(int j=0;j<radius_num_vector.size();++j)
        {
             myfile<<bfs_runtimes_vector[i][j].count()<<" ";
        }
        myfile<<std::endl;
    }

    myfile<<std::endl;

    for(int i=0;i<cns_runtimes_vector.size();++i)
    {
        for(int j=0;j<radius_num_vector.size();++j)
        {
             myfile<<cns_runtimes_vector[i][j].count()<<" ";
        }
        myfile<<std::endl;
    }

    myfile.close();

    return 0;
}
*/
