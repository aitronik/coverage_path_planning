cmake_minimum_required(VERSION 3.5)
project(coverage_path_lib)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()


find_package(ament_cmake REQUIRED)

find_package( CGAL QUIET COMPONENTS  )
find_package(CGAL COMPONENTS Qt5)
find_package( OpenCV REQUIRED  )
find_package( Boost REQUIRED )


include_directories(include)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_library(lib_coverage_path2 src/utils.cpp src/CoveragePathCreator.cpp src/CoveragePlotHelper.cpp src/pathPoints.cpp src/CgalPlotHelper.cpp src/PolygonCreator.cpp src/CoveragePath.cpp src/MyDecomposition.cpp)
ament_export_targets(export_lib_coverage_path2 HAS_LIBRARY_TARGET) 

# Link the executable to CGAL and third-party libraries
target_link_libraries(lib_coverage_path2 PRIVATE CGAL::CGAL ${OpenCV_LIBRARIES})

install(
  TARGETS lib_coverage_path2  
  EXPORT export_lib_coverage_path2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
) 
ament_export_libraries(lib_coverage_path2)

add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)

#add_executable( coverage_path main.cpp decomposer.cpp generatorPath.cpp utils.cpp )
add_executable(coverage_path_node src/runCoveragePath.cpp)
#target_link_libraries(coverage_path lib_coverage_path2 ${OpenCV_LIBRARIES})
target_link_libraries(coverage_path_node lib_coverage_path2 ${OpenCV_LIBRARIES} GTest::gtest_main)
target_link_libraries(lib_coverage_path2 PUBLIC CGAL::CGAL_Qt5)

include(GoogleTest)

install(TARGETS      coverage_path_node
        DESTINATION  lib/coverage_path_lib)

 
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".git*" EXCLUDE
)


ament_export_include_directories(include)
#ament_export_dependencies(${dependencies})
        
ament_package()
