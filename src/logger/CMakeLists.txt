add_library(logger STATIC
    Logger.cpp
    ImageLogger.cpp
)

target_include_directories(logger PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(logger PUBLIC
    spdlog::spdlog_header_only
    SPSCQueue
    RingBuffer
)
