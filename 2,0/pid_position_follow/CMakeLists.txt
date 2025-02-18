cmake_minimum_required(VERSION 3.8)
project(pid_position_follow)

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()


include_directories(
  ${PROJECT_SOURCE_DIR}/include/pid_position_follow
  ${PROJECT_SOURCE_DIR}/include/cubic_spline
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pid_position_follow.cpp
  src/pid.cpp
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN pid_position_follow::RobotCtrl
  EXECUTABLE pid_position_follow_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()


# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_auto_package()

