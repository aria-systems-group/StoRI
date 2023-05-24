cmake_minimum_required(VERSION 3.0)

# set the project name
project(STL_RRT VERSION 1.0)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# find my packages hopefully
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

# add my libraries (extra functions)
add_subdirectory(stl_rrt_planner)
add_subdirectory(PrSTL_Monitor)
add_subdirectory(Spaces)

# add the executable
add_executable(STL_RRT main.cpp) 

# add directories to list of paths to search for include filesa
target_include_directories(STL_RRT PRIVATE
${OMPL_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/stl_rrt_planner
${PROJECT_SOURCE_DIR}/PrSTL_Monitor
${PROJECT_SOURCE_DIR}/Spaces
)

# link the libraries
target_link_libraries(STL_RRT
${OMPL_LIBRARIES}
Eigen3::Eigen
stlrrt
prstl
matState
)
