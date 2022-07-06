set(controller_SRC
  HelpUpController.cpp
  utils/ComputationPoint.cpp
  utils/PointProjector.cpp
  utils/CoMQP.cpp
  states/PostAdd.cpp
  # Tasks/BoundCoMAcceleration.cpp
  # Tasks/BoundCoMVelocity.cpp
)

set(controller_HDR
  HelpUpController.h
  utils/ComputationPoint.h
  utils/PointProjector.h
  utils/CoMQP.h
  states/PostAdd.h
  # Tasks/BoundCoMAcceleration.h
  # Tasks/BoundCoMVelocity.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DHelpUpController_EXPORTS")
target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
  mc_rtc::mc_control_fsm
  Stabiliplus::ProblemDescriptor
  Stabiliplus::StabilityPolytope
  eigen-quadprog::eigen-quadprog
  sch-core::sch-core
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
add_subdirectory(states)