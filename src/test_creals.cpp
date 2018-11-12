#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include <SPHSimulator.hpp>
#include "simulation.hpp"
using namespace std;



// basically I should add a time value for the SPHSimulator!!!
// and maybe...if

// or just for each frame, build a new file.
// tomorrow, build my own visualization class.
void generate_particle()  // maybe I should directly put all particles in all frame to one vector...
{
    SPHSimulator test_simulator(1.0,0.01,1);
    test_simulator.set_N(1);
    test_simulator.generate_particles();

    test_simulator.set_file_path("sim_result/");
    test_simulator.output_particles_xml();

    for(int i=0;i<100;++i)
    {
        test_simulator.update_freefall_motion();
        test_simulator.output_particles_xml();
    }
    return;
}

void generate_sim_rec()  // maybe I should directly put all particles in all frame to one vector...
{
    // a for loop to generate every thing, and then run...
    SPHSimulator test_simulator(0.5f,0.1,10);
    for(int i=0;i<100;++i)
    {
        test_simulator.update_two_cubes_collision();
        test_simulator.update_sim_record_state();
        std::cout<<"iteration "<< i <<std::endl;
    }

    test_simulator.output_sim_record_bin("two_cubes_dt_0_1.bin");

    return;
}

int main()
{
//    SPHSimulator test_simulator(1.0,0.1,1);
//    test_simulator.set_N(1);
//    test_simulator.generate_particles();
//    std::cout<< "now print set 1"<<std::endl;
//    test_simulator.print_all_particles();
//    std::cout<<"total size is "<< test_simulator.get_number_of_particles()<<std::endl;

//    std::string file_path ="simulator.xml";

//    std::ofstream myfile(file_path);   //maybe here, this ofstream could be useful... it should has a continuse read mode!
//    {
//    cereal::XMLOutputArchive output(myfile); // stream to cout
//    output(test_simulator);  //not good... maybe directly ar the vector
//    }

//    test_simulator.update_freefall_motion();

//    {
//    cereal::XMLOutputArchive output(myfile); // stream to cout
//    output(test_simulator);  //not good... maybe directly ar the vector
//    }

//    myfile.close();

//    std::ifstream file(file_path);
//    SPHSimulator simulator2(1.0,0.1,1);
//    SPHSimulator simulator3(1.0,0.1,1);
//    //test_simulator.set_N(1);
//    {
//    cereal::XMLInputArchive input(file); // stream to cout
//    input(simulator2);  //not good... maybe directly ar the vector
//    }
//    std::cout<<"fuck1"<<std::endl;
//    simulator2.print_all_particles();

    // now we try to continuely read it..doesn't work...
////    {
////    cereal::XMLInputArchive input(file); // stream to cout
////    input(simulator2);  //not good... maybe directly ar the vector
////    }
//    std::cout<<"fuck2"<<std::endl;

//    simulator2.print_all_particles();

////    test_simulator.set_file_path("sim_result/");
////    test_simulator.output_particles_xml();
////    test_simulator.update_freefall_motion();
////    test_simulator.output_particles_xml();


//    SPHSimulator test_simulator_2(1.0,0.1,1);
//    test_simulator_2.set_N(1);
//    test_simulator_2.set_file_path("sim_result/");
//    test_simulator_2.input_particles_xml();
//    test_simulator_2.print_all_particles();
//    test_simulator_2.input_particles_xml();
//    test_simulator_2.print_all_particles();


    //    Simulation simulation(0.5f, 0.01);
    //    simulation.set_file_path("sim_result/");
//    simulation.input_particles_xml();
//    simulation.input_particles_xml();
//    simulation.update_from_file();
//    simulation.update_from_file();



    //generate_particle();
    generate_sim_rec();

    return 0;

}





//// type support
//#include <cereal/types/map.hpp>
//#include <cereal/types/vector.hpp>
//#include <cereal/types/string.hpp>
//#include <cereal/types/complex.hpp>

//// for doing the actual serialization
//#include <cereal/archives/json.hpp>
//#include <cereal/archives/binary.hpp>

//#include <iostream>

//class Stuff
//{
//  public:
//    Stuff() = default;

//    Stuff(double x1,double x2)
//    {
//        data.push_back(x1);
//        data.push_back(x2);
//    }


//    Stuff(double x1,double x2,double x3)
//    {
//        data.push_back(x1);
//        data.push_back(x2);
//        data.push_back(x3);

//    }



//    std::vector<double> data;

//    friend class cereal::access;

//    template <class Archive>
//    void serialize( Archive & ar )
//    {
//      ar( CEREAL_NVP(data) );
//    }
//};

//int main()
//{

//  Stuff myStuff1(1.0,2.0);
//  Stuff myStuff2(3.0,4.0,5.0);


//  std::stringstream ss; // any stream can be used


//  {
//      cereal::BinaryOutputArchive output(ss); // stream to cout
//      output( cereal::make_nvp("best data ever", myStuff1) );
//  }
//  {
//      cereal::BinaryOutputArchive output(ss); // stream to cout
//      output( cereal::make_nvp("best data ever", myStuff2) );
//  }

//  Stuff mystuff3,mystuff4;

//  {
//      cereal::BinaryInputArchive input(ss);
//      input(cereal::make_nvp("best data ever", mystuff3),cereal::make_nvp("best data ever", mystuff4));
//  }


////  {
////      cereal::JSONInputArchive input(ss);
////      input(cereal::make_nvp("best data ever", mystuff4));
////  }

//  std::cout<<mystuff3.data[0]<<mystuff4.data[0]<<std::endl;



//    int x(1),y(2),z,t;
//    std::stringstream ss;
//    {
//          cereal::BinaryOutputArchive output(ss); // stream to cout
//          output( x );
//      }
//      {
//          cereal::BinaryOutputArchive output(ss); // stream to cout
//          output( y );
//      }
//      {
//          cereal::BinaryInputArchive input(ss);
//          input(z,t);
//      }
//    std::cout<<z<<t<<std::endl;
//}
