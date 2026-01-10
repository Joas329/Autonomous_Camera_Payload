# Create communicator static library
add_library(communicator STATIC
    SoberApi.cpp
    # Communicator.cpp
    # SoberProtocol.cpp
)

# Crow include path — header-only library
target_include_directories(communicator
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/3rdparty/c-periphery/src
        ${CMAKE_SOURCE_DIR}/3rdparty/Crow/include
)

# Add defines
target_compile_definitions(communicator
    PUBLIC
        ROOT=\"${CMAKE_SOURCE_DIR}\"
)

# Link other project libraries
target_link_libraries(communicator
    PRIVATE
        logger
        manager
    PUBLIC
        RingBuffer
)
