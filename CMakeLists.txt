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

set(AROBASE "@")
configure_file(etc/HelpUpController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/HelpUpController.yaml")
configure_file(etc/trajectories/log-to-datastore.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/trajectories/log-to-datastore.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/HelpUpController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/trajectories/log-to-datastore.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")
# install(FILES etc/trajectories/standup.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")
# install(FILES etc/trajectories/perturbed_stands_no_suit/log1.bin etc/trajectories/perturbed_stands_no_suit/log2.bin etc/trajectories/perturbed_stands_no_suit/log3.bin etc/trajectories/perturbed_stands_no_suit/log4.bin etc/trajectories/perturbed_stands_no_suit/log5.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories/perturbed_stands_no_suit")
# install(FILES etc/force_shoes/forceshoes1.bin etc/force_shoes/forceshoes2.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")
# install(FILES etc/force_shoes/F1.yaml etc/force_shoes/F2.yaml etc/force_shoes/F3.yaml etc/force_shoes/F4.yaml etc/force_shoes/F5.yaml etc/force_shoes/F6.yaml etc/force_shoes/F7.yaml etc/force_shoes/F8.yaml DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/forces")
# install(FILES etc/force_shoes/forceshoes_celia.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")
# install(FILES etc/force_shoes/celia/celia_nosuit1.bin etc/force_shoes/celia/celia_nosuit2.bin etc/force_shoes/celia/celia_nosuit3.bin etc/force_shoes/celia/celia_suit1.bin etc/force_shoes/celia/celia_suit2.bin etc/force_shoes/celia/celia_suit3.bin DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories")

install(FILES etc/force_shoes/celia/tests-17nov/nosuit1.bin etc/force_shoes/celia/tests-17nov/nosuit2.bin etc/force_shoes/celia/tests-17nov/nosuit3.bin
              etc/force_shoes/celia/tests-17nov/suit1.bin etc/force_shoes/celia/tests-17nov/suit2.bin etc/force_shoes/celia/tests-17nov/suit3.bin
              etc/force_shoes/celia/tests-17nov/suit4.bin etc/force_shoes/celia/tests-17nov/suit5.bin
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/trajectories/celia17nov/")
