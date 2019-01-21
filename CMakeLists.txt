cmake_minimum_required(VERSION 3.1)

project(simulator)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# set debug flag
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_BUILD_TYPE Release)
# Setting EIGEN3_ROOT here lets us override which Eigen library merely3d chooses
set(EIGEN3_ROOT ${PROJECT_SOURCE_DIR}/extern/eigen_3.3.4)

# Setting CEREALS_ROOT here
set(CEREALS_ROOT ${PROJECT_SOURCE_DIR}/extern/cereal/include)

set(CLI11_ROOT ${PROJECT_SOURCE_DIR}/extern/CLI11/include/CLI11)

if (UNIX)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif(OPENMP_FOUND)
endif (UNIX)

add_subdirectory(extern/merely3d)
add_subdirectory(extern/CompactNSearch)


set(TINY_OBJ_LOADER_INCLUDE ${CMAKE_SOURCE_DIR}/extern/tinyobjloader/include)

set(CLI11_FILES
		${PROJECT_SOURCE_DIR}/extern/CLI11/include/CLI11/CLI11.hpp)

set(IMGUI_FILES
        src/imgui/imgui.h
        src/imgui/imgui.cpp
        src/imgui/imconfig.h
        src/imgui/imgui_draw.cpp
        src/imgui/imgui_internal.h
        src/imgui/stb_rect_pack.h
        src/imgui/stb_textedit.h
        src/imgui/stb_truetype.h
        src/imgui/imgui_demo.cpp
        src/imgui/imgui_event_handler.h
        src/imgui/imgui_event_handler.cpp)

set(UTIL_FILES
        src/util/obj.hpp
        src/util/obj.cpp
        src/util/triangle_mesh.hpp
        src/util/triangle_mesh.cpp)

set(SOURCE_FILES
        ${CLI11_FILES}
        ${IMGUI_FILES}
        ${UTIL_FILES}
        src/math_types.hpp
        src/simulation.hpp
        src/simulation.cpp
        src/NeighborSearcher.hpp
        src/NeighborSearcher.cpp
        src/KernelHandler.hpp
        src/KernelHandler.cpp
        src/SPHSimulator.hpp
        src/SPHSimulator.cpp
        src/Particle.hpp
        src/ParticleFunc.hpp
        src/ParticleFunc.cpp
        src/ParticleGenerator.hpp
        src/ParticleGenerator.cpp
        src/sim_record.hpp
        src/mesh_record.hpp
)


set(DERIVED_CLASS_FOLDER src/derived_class)
set(DERIVED_CLASS_FILES
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_2cubes.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_free_fall_motion.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_rigid_body.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_dam_breaking.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_double_dam_breaking.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_dam_breaking_thin.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_drop_center.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_drop_on_water.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_fluid_pillar.hpp
    ${DERIVED_CLASS_FOLDER}/SPHSimulator_pbf.hpp
   
# when we have new simulation scenero, we do derive it from sphsimulator class, and add file here.
)

set(VISUAL_SOURCE_FILES
		${CLI11_FILES}
        ${IMGUI_FILES}
        ${UTIL_FILES}
        src/math_types.hpp
        src/visual.hpp
        src/visual.cpp
        src/sim_record.hpp
        src/Particle.hpp
        src/mesh_record.hpp
)

# Compile source files into a static lib, so that we don't have to compile source files
# twice for our main executable and our unit tests
add_library(simulator_lib STATIC ${SOURCE_FILES} ${DERIVED_CLASS_FILES})
target_link_libraries(simulator_lib merely3d CompactNSearch)
target_include_directories(simulator_lib PUBLIC ${CMAKE_SOURCE_DIR}/src ${TINY_OBJ_LOADER_INCLUDE} ${CEREALS_ROOT} ${CLI11_ROOT} ${DERIVED_CLASS_FOLDER})

# simulator is our visualization node.
add_executable(simulator src/main.cpp)
target_link_libraries(simulator simulator_lib )
target_include_directories(simulator PRIVATE ${CEREALS_ROOT} ${CLI11_ROOT})


add_library(sim_visual_lib STATIC ${VISUAL_SOURCE_FILES} )
target_link_libraries(sim_visual_lib merely3d CompactNSearch)
target_include_directories(sim_visual_lib PUBLIC ${CMAKE_SOURCE_DIR}/src ${TINY_OBJ_LOADER_INCLUDE} ${CEREALS_ROOT} ${CLI11_ROOT})

add_executable(visualizer src/visualizer.cpp)
target_link_libraries(visualizer sim_visual_lib)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION ${CMAKE_BINARY_DIR})

# # Add source files for unit tests here.
set(TEST_FILES tests/sample_tests.cpp)
set(KERNEL_TEST_FILES tests/kernel_tests.cpp)

add_executable(simulator_test tests/testmain.cpp ${TEST_FILES})
target_link_libraries(simulator_test simulator_lib)

add_executable(kernel_test tests/testmain.cpp ${KERNEL_TEST_FILES})
target_link_libraries(kernel_test simulator_lib)

# # merely3d already ships with Catch for unit testing, so let's just use the same
target_include_directories(simulator_test PRIVATE extern/merely3d/extern/catch )
target_include_directories(kernel_test PRIVATE extern/merely3d/extern/catch )

add_executable(save_simulation src/save_simulation.cpp)
target_link_libraries(save_simulation simulator_lib)
target_include_directories(save_simulation PRIVATE ${EIGEN3_ROOT} ${CEREALS_ROOT} ${CLI11_ROOT})


set(MARCHING_CUBE_LIB_FILES
        ${CLI11_FILES}
        ${IMGUI_FILES}
        src/marching_cube.cpp
        src/marching_cube.hpp
        src/marching_cubes_lut.hpp
        src/derived_class/marching_cube_torus.hpp
        src/derived_class/marching_cube_sphere.hpp
        src/derived_class/marching_cube_fluid.hpp
)

add_library(marching_cube_lib STATIC ${MARCHING_CUBE_LIB_FILES} ${VISUAL_SOURCE_FILES})
target_link_libraries(marching_cube_lib merely3d)
target_include_directories(marching_cube_lib PUBLIC
                            ${CMAKE_SOURCE_DIR}/src
                            ${CLI11_ROOT}
                            ${EIGEN_ROOT}
                            ${CMAKE_SOURCE_DIR}/src/derived_class
                            ${TINY_OBJ_LOADER_INCLUDE}
                            ${CEREALS_ROOT}
    )

add_executable(test_marching_cube src/test_marching_cube.cpp)
target_link_libraries(test_marching_cube marching_cube_lib)
#target_include_directories(test_marching_cube PUBLIC ${CMAKE_SOURCE_DIR}/src )


add_executable(fluid_rendering_preview src/render_preview.cpp)
target_link_libraries(fluid_rendering_preview CompactNSearch merely3d  simulator_lib marching_cube_lib)
target_include_directories(fluid_rendering_preview PRIVATE ${EIGEN3_ROOT} ${EIGEN_ROOT} ${CEREALS_ROOT} ${CLI11_ROOT})

add_executable(save_fluid_mesh src/save_fluid_mesh.cpp)
target_link_libraries(save_fluid_mesh CompactNSearch merely3d  simulator_lib marching_cube_lib)
target_include_directories(save_fluid_mesh PRIVATE ${EIGEN3_ROOT} ${EIGEN_ROOT} ${CEREALS_ROOT} ${CLI11_ROOT})

add_executable(display_fluid_mesh src/display_fluid_mesh.cpp)
target_link_libraries(display_fluid_mesh CompactNSearch merely3d  simulator_lib marching_cube_lib)
target_include_directories(display_fluid_mesh PRIVATE ${EIGEN3_ROOT} ${EIGEN_ROOT} ${CEREALS_ROOT} ${CLI11_ROOT})