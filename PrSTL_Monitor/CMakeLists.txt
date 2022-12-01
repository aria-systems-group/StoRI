add_library(prstl PrSTL_Monitor.cpp)

target_include_directories(prstl PUBLIC
${PROJECT_SOURCE_DIR}/PrSTL_Monitor
)

# target_link_libraries(stlsst
#    ${OMPL_LIBRARIES}
# )