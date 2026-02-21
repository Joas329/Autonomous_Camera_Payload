set(EBUS_SDK_INCLUDE_DIR /opt/pleora/ebus_sdk/linux-aarch64-arm/include)
set(EBUS_SDK_LIB_DIR     /opt/pleora/ebus_sdk/linux-aarch64-arm/lib)

add_library(camera STATIC
    LastFrame.cpp
    Ticker.cpp
    IRCamera.cpp
    TelopsCamera.cpp
    OpticalSensor.cpp

    # Telops + eBUS sources
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Telops/SerialCommunicationManager.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Telops/OperationalCommandRead.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Telops/OperationalCommandWrite.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Telops/ReadInternalLensTemperature.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Telops/PowerSuppliesRead.cpp

    # MANDATORY for FailureResponse symbols
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/eBUS_SDK/FailureResponse.cpp
)

target_include_directories(camera PUBLIC
    ${CMAKE_SOURCE_DIR}/include

    # Pleora SDK include directory
    ${EBUS_SDK_INCLUDE_DIR}

    # Telops headers
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Telops
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/include/Protocol

    # *** THIS FIX WAS MISSING ***
    ${CMAKE_SOURCE_DIR}/3rdparty/eBUS-telops/eBUS_SDK
)

target_link_directories(camera PRIVATE ${EBUS_SDK_LIB_DIR})

target_link_libraries(camera PRIVATE
    ${EBUS_SDK_LIB_DIR}/libPvBase.so
    ${EBUS_SDK_LIB_DIR}/libPvDevice.so
    ${EBUS_SDK_LIB_DIR}/libPvSerial.so
    ${EBUS_SDK_LIB_DIR}/libPvStream.so
    ${EBUS_SDK_LIB_DIR}/libPvBuffer.so
    ${EBUS_SDK_LIB_DIR}/libPvGenICam.so
    ${EBUS_SDK_LIB_DIR}/libPvSystem.so
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
