cmake_minimum_required(VERSION 3.5)
project(tf_broadcaster)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


# 编译可执行文件
add_executable(robot_odometer src/robot_odometer.cpp)

# 链接 ROS 2 库
ament_target_dependencies(robot_odometer
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# 安装可执行文件
install(TARGETS
  robot_odometer
  DESTINATION lib/${PROJECT_NAME}
)



# 安装 launch 文件到安装目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 config 文件夹到安装目录
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
