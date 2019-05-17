add_library( myslam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    vo.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
