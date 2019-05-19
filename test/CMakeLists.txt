add_executable( run_vo run_vo.cpp )
target_link_libraries( run_vo monovo )
add_executable(run_simplevo run_simplevo.cpp)
target_link_libraries(run_simplevo monovo)

add_executable(run_vo_track run_vo_track.cpp)
target_link_libraries(run_vo_track monovo)