add_library(stlrrt stl_rrt.cpp)

target_include_directories(stlrrt PUBLIC
${OMPL_INCLUDE_DIRS} 
${PROJECT_SOURCE_DIR}/stl_rrt_planner
${PROJECT_SOURCE_DIR}/PrSTL_Monitor
${PROJECT_SOURCE_DIR}/Spaces
)

target_link_libraries(stlrrt
    ${OMPL_LIBRARIES}
)