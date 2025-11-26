add_library(health STATIC HealthMonitor.cpp)

target_include_directories(health PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(health PRIVATE eventbus)