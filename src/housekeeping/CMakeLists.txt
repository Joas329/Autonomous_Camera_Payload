add_library(housekeeping STATIC icm45605.cpp BME280.cpp LIS3MDLTR.cpp systemInterface.cpp Ina237.cpp Max31725.cpp Thermocouple.cpp)

target_include_directories(housekeeping PUBLIC ${CMAKE_SOURCE_DIR}/include
                                            ${CMAKE_SOURCE_DIR}/3rdparty/icm45605TDK/examples
                                            ${CMAKE_SOURCE_DIR}/3rdparty/icm45605TDK/icm45605
)

target_link_libraries(housekeeping PRIVATE eventbus logger icm45605TDK)