add_library(camera STATIC
    OpticalCamera.cpp
    LastFrame.cpp
    Ticker.cpp
    IRCamera.cpp
    TelopsCamera.cpp
)

target_include_directories(camera PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/camera
    ${CMAKE_SOURCE_DIR}/3rdparty/RingBuffer

    # Telops SDK full include set
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Telops
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Protocol
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/Private

    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/eBUS_SDK
)

target_link_libraries(camera PUBLIC
    RingBuffer
    SPSCQueue
    logger
    spdlog::spdlog_header_only
    lz4::lz4
    ebus
    Spinnaker::Spinnaker
    turbojpeg
)
