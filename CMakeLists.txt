add_library(stori StoRI_Monitor.cpp)

find_package(Eigen3 REQUIRED)

target_include_directories(stori PUBLIC
  ${PROJECT_SOURCE_DIR}/StoRI_Monitor
  ${Boost_INCLUDE_DIRS}
)
# $ENV{EIGEN3_INCLUDE_DIR}
# )

# target_link_libraries(prstl
# Eigen3::Eigen
# )
# target_link_libraries(stlsst
#    ${OMPL_LIBRARIES}
# )
