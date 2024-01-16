cmake_minimum_required(VERSION 3.5)

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
find_package(mc_panda REQUIRED)

find_package(XsensPlugin REQUIRED)

add_subdirectory(src)
add_subdirectory(modules)
add_subdirectory(plugins)

set(AROBASE "@")
configure_file(etc/HelpUpController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/HelpUpController.yaml")
configure_file(etc/trajectories/log-to-datastore.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/trajectories/log-to-datastore.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/HelpUpController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/trajectories/log-to-datastore.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")

install(FILES etc/force_shoes/celia/tests-17nov/nosuit1.bin etc/force_shoes/celia/tests-17nov/nosuit2.bin etc/force_shoes/celia/tests-17nov/nosuit3.bin
              etc/force_shoes/celia/tests-17nov/suit1.bin etc/force_shoes/celia/tests-17nov/suit2.bin etc/force_shoes/celia/tests-17nov/suit3.bin
              etc/force_shoes/celia/tests-17nov/suit4.bin etc/force_shoes/celia/tests-17nov/suit5.bin
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories/celia17nov/")

# install(FILES etc/force_shoes/celia/test-perturb-03jul/QP_alone_2023-07-03.bin etc/force_shoes/celia/test-perturb-03jul/QP_perturbed_2023-07-03.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories/celia03jul/")
# install(FILES etc/force_shoes/hugo/18dec2023.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories/hugo/")
install(FILES etc/force_shoes/hugo/NoSuitHelpBack.bin etc/force_shoes/hugo/NoSuitHelpFront.bin etc/force_shoes/hugo/NoSuitNormal1.bin etc/force_shoes/hugo/NoSuitNormal2.bin
              etc/force_shoes/hugo/SuitHelpBack.bin etc/force_shoes/hugo/SuitHelpFront.bin etc/force_shoes/hugo/SuitNormal1.bin etc/force_shoes/hugo/SuitNormal2.bin
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories/hugo/")
