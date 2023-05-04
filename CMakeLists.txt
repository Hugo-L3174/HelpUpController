cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME HelpUpController)
set(PROJECT_DESCRIPTION "HelpUpController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)
find_package(Stabiliplus REQUIRED)
find_package(eigen-quadprog REQUIRED)
find_package(sch-core REQUIRED)
find_package(gram_savitzky_golay REQUIRED)


add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/HelpUpController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/HelpUpController.yaml")
configure_file(etc/trajectories/log-to-datastore.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/trajectories/log-to-datastore.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/HelpUpController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/trajectories/log-to-datastore.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")
install(FILES etc/trajectories/standup.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")

