add_library(prstl PrSTL_Monitor.cpp)

find_package(Eigen3 REQUIRED)

target_include_directories(prstl PUBLIC
${PROJECT_SOURCE_DIR}/PrSTL_Monitor
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
