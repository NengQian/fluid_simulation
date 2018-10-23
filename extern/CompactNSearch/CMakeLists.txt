cmake_minimum_required(VERSION 3.2)

project(CompactNSearch)

# Visual studio solution directories.
set_property(GLOBAL PROPERTY USE_FOLDERS on)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (UNIX)
find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)
endif (UNIX)

set (HEADER_FILES
		include/CompactNSearch/Config.h
		include/CompactNSearch/CompactNSearch.h
		include/CompactNSearch/PointSet.h
		include/CompactNSearch/DataStructures.h)

add_library(CompactNSearch
		${HEADER_FILES}
		src/CompactNSearch.cpp
)

target_include_directories(CompactNSearch PUBLIC include)

install(FILES "include/CompactNSearch/CompactNSearch" ${HEADER_FILES}
	DESTINATION include/)

install(TARGETS CompactNSearch
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib)
	
OPTION(USE_DOUBLE_PRECISION "Use double precision"	ON)
if (USE_DOUBLE_PRECISION)
	add_definitions( -DUSE_DOUBLE)
    # INTERFACE is necessary so that projects using CompactNSearch header files
    # also get the right definitions
    target_compile_definitions(CompactNSearch INTERFACE -DUSE_DOUBLE)
endif (USE_DOUBLE_PRECISION)

option(BUILD_DEMO "Build example of how to use this library."
		ON)
if(BUILD_DEMO)
	add_subdirectory(demo)
endif(BUILD_DEMO)

