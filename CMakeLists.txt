cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(line)
 
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
 
add_definitions(${PCL_DEFINITIONS})

set(SRC_LIST
	 ./line.cpp
	 ./line_projection_factor.cpp
	 ./pose_local_parameterization.cpp
     ./utility
   )

add_executable(line ${SRC_LIST})
#add_executable (ceres_line ceres_line.cpp)
 
target_link_libraries (line ${PCL_LIBRARIES}
${CERES_LIBRARIES} 
${EIGEN3_LIBS})
