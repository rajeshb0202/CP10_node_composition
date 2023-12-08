cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(attach_shelf REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(preapproach SHARED src/preapproach.cpp)
target_compile_definitions(preapproach PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(preapproach
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs")
rclcpp_components_register_nodes(preapproach "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:preapproach>\n")


add_library(attach_server SHARED src/attach_server.cpp)
target_compile_definitions(attach_server PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "attach_shelf")
rclcpp_components_register_nodes(attach_server "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:attach_server>\n")



install(TARGETS
  preapproach
  attach_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(manual_composition  src/manual_composition.cpp)
target_link_libraries(manual_composition attach_server )
ament_target_dependencies(manual_composition  "rclcpp" "geometry_msgs" "attach_shelf" "nav_msgs"  "sensor_msgs" "tf2_ros")

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME})



ament_package()
