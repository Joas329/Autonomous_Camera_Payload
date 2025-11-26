add_library(communicator STATIC Communicator.cpp SoberProtocol.cpp)

target_include_directories(communicator PUBLIC ${CMAKE_SOURCE_DIR}/include 
                             ${CMAKE_SOURCE_DIR}/3rdparty/c-periphery/src
)

target_compile_definitions(communicator
  PUBLIC
    ROOT=\"${CMAKE_SOURCE_DIR}\"
)

target_link_libraries(communicator PRIVATE eventbus logger periphery
PUBLIC
    RingBuffer
)