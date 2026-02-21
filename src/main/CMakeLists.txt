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


# ---- Copy UI assets into repo runtime folder: <repo>/bin/ui ----
get_filename_component(SOBER_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(UI_RUNTIME_DIR "${SOBER_REPO_ROOT}/bin/ui")

add_custom_command(
  TARGET sober_main_app POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UI_RUNTIME_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${SOBER_REPO_ROOT}/src/ui/control_panel.html"
          "${UI_RUNTIME_DIR}/control_panel.html"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${SOBER_REPO_ROOT}/src/ui/update_base_from_nmcli.sh"
          "${UI_RUNTIME_DIR}/update_base_from_nmcli.sh"
  COMMAND chmod +x "${UI_RUNTIME_DIR}/update_base_from_nmcli.sh"
  VERBATIM
)
