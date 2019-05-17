include_directories( ${PROJECT_SOURCE_DIR}/include )
add_executable( run_vo run_vo.cpp )
target_link_libraries( run_vo myslam )