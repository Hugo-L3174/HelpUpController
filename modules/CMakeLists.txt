configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")
add_robot(PandaHelpUp module.cpp module.h)
target_link_libraries(PandaHelpUp PUBLIC mc_panda::panda)
target_include_directories(PandaHelpUp PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

install(DIRECTORY rsdf/
  DESTINATION "${PANDA_DESCRIPTION_PATH}/rsdf/panda_helpup"
  FILES_MATCHING PATTERN "*.rsdf")
