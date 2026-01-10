add_executable(sober_main_app main.cpp)

target_link_options(sober_main_app PRIVATE
  -Wl,-rpath,/opt/spinnaker/lib
  -Wl,-rpath-link,${SPINNAKER_ROOT}/lib
)

# minimum set of libs needed for a pure-camera build
target_link_libraries(sober_main_app
  PRIVATE
    manager
    camera
    communicator
    logger
    spdlog::spdlog_header_only

    # dynamic libs
    Spinnaker::Spinnaker
    pthread
    dl
    rt
)
install(
    TARGETS sober_main_app
    RUNTIME DESTINATION ${SOBER_INSTALL_ROOT}
)