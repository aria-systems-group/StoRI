add_library(matState R2BeliefSpace.cpp)

target_include_directories(matState PUBLIC
${OMPL_INCLUDE_DIRS} 
${PROJECT_SOURCE_DIR}/Spaces
)

target_link_libraries(matState
    ${OMPL_LIBRARIES}
    Eigen3::Eigen
)