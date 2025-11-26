add_library(manager STATIC
    CameraManager.cpp
    SystemController.cpp
    CameraSynchronizer.cpp
)

target_include_directories(manager PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(manager
    PUBLIC
        camera
        logger
)
