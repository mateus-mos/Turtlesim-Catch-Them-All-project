cmake_minimum_required(VERSION 3.5)
project(turtlesim_catch_them_all_cpp)

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
find_package(turtlesim REQUIRED)
find_package(turtlesim_catch_them_all_project_interfaces REQUIRED)


add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim turtlesim_catch_them_all_project_interfaces) 

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp turtlesim_catch_them_all_project_interfaces) 

install(TARGETS
  turtle_spawner
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
