cmake_minimum_required(VERSION 2.8.3)
project(joystick_state_controller)

include("../cmake_modules/CMakeOpt.cmake")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS realtime_tools roscpp
hardware_interface pluginlib controller_interface sensor_msgs joystick_interface
 std_msgs message_generation)

add_message_files (
  FILES
  JoystickState.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
# Declare catkin package
catkin_package(
  CATKIN_DEPENDS realtime_tools roscpp hardware_interface pluginlib
  controller_interface sensor_msgs message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/joystick_state_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## joystick controller messages to be built before the
## joystick controller code tries to use them
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#
#  add_rostest_gtest(joystick_state_controller_test
#                    test/joystick_state_controller.test
#                    test/joystick_state_controller_test.cpp)
#  target_link_libraries(joystick_state_controller_test ${PROJECT_NAME})
#endif()
# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES joystick_state_plugin.xml
              joystick_state_controller.yaml
              joystick_state_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
