cmake_minimum_required(VERSION 3.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

add_library(stori StoRI_Monitor.cpp)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

target_include_directories(stori PUBLIC
  ${PROJECT_SOURCE_DIR}/StoRI_Monitor
  ${Boost_INCLUDE_DIRS}
)
