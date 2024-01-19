set(controller_SRC
  HelpUpController.cpp
  MCStabilityPolytope.cpp
  utils/PointProjector.cpp
  utils/DCM_VRPtracker.cpp
  Tasks/ForceConstrainedTransformTask.cpp
)

set(controller_HDR
  HelpUpController.h
  MCStabilityPolytope.h
  utils/PointProjector.h
  utils/DCM_VRPtracker.h
  Tasks/TrackDesiredForceTask.h
  Tasks/ForceConstrainedTransformTask.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DHelpUpController_EXPORTS")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  mc_rtc::mc_control_fsm
  mc_panda::devices
  Stabiliplus::ProblemDescriptor
  Stabiliplus::StabilityPolytope
  eigen-quadprog::eigen-quadprog
  sch-core::sch-core
  gram_savitzky_golay::gram_savitzky_golay
  )
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  )
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
configure_file(../etc/force_shoes/config.in.h "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")
target_include_directories(HelpUpController PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

add_subdirectory(states)
