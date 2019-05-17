add_library( monovo SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    vo.cpp
)

target_link_libraries( monovo
    ${THIRD_PARTY_LIBS}
)
