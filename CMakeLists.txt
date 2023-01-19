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
find_package( OpenCV REQUIRED  )
find_package( Boost REQUIRED )


include_directories(include)

add_library(lib_coverage_path src/utils.cpp src/CoveragePathCreator.cpp src/CoveragePlotHelper.cpp src/MyDecomposition.cpp)
ament_export_targets(export_lib_coverage_path HAS_LIBRARY_TARGET) 

# Link the executable to CGAL and third-party libraries
target_link_libraries(lib_coverage_path PRIVATE CGAL::CGAL ${OpenCV_LIBRARIES})

install(
  TARGETS lib_coverage_path  
  EXPORT export_lib_coverage_path
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
) 
ament_export_libraries(lib_coverage_path)


#add_executable( coverage_path main.cpp generatorPath.cpp utils.cpp mydecomposition.cpp )
add_executable(coverage_path src/main.cpp)
target_link_libraries(coverage_path lib_coverage_path ${OpenCV_LIBRARIES})


install(TARGETS      coverage_path
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