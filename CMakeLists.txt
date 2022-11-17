# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.23)

project( coverage_path )

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )



if (NOT CMAKE_CXX_STANDARD) 
  set(CMAKE_CXX_STANDARD 17)
endif ()


if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# opencv and its components
find_package( OpenCV REQUIRED  )

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# include for local directory

# include for local package


# Creating entries for target: decomposer
# ############################

#add_executable( coverage_path main.cpp decomposer.cpp generatorPath.cpp utils.cpp)
add_executable(coverage_path main.cpp utils.cpp CoveragePathCreator.cpp CoveragePlotHelper.cpp )

# add_to_cached_list( CGAL_EXECUTABLE_TARGETS decomposer )

# Link the executable to CGAL and third-party libraries
target_link_libraries(coverage_path PRIVATE CGAL::CGAL ${OpenCV_LIBRARIES})

