cmake_minimum_required(VERSION 3.14)
project(lqr_lateral_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/lqr_lateral_controller/lqr_lateral_controller.hpp
  include/lqr_lateral_controller/visibility_control.hpp
  src/lqr_lateral_controller.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/lqr_lateral_controller/lqr_lateral_controller_node.hpp
  src/lqr_lateral_controller_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "lqr_lateral_controller::LqrLateralControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_lqr_lateral_controller
    test/test_lqr_lateral_controller.cpp
  )
  ament_target_dependencies(test_lqr_lateral_controller ${TEST_SOURCES})
  target_link_libraries(test_lqr_lateral_controller ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
