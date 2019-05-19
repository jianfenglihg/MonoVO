add_library( monovo SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    localmap.cpp
    vo.cpp
    simplevo.cpp
    vo_track.cpp
)

target_link_libraries( monovo
    ${THIRD_PARTY_LIBS}
)
