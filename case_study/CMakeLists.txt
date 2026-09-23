cmake_minimum_required(VERSION 3.16)
project(hierarchical_control_case_study LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface controller_manager hardware_interface hierarchical_control
  pluginlib rclcpp rclcpp_lifecycle std_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(case_study_controllers SHARED
  src/travel_registry.cpp
  src/wheel_controller.cpp
  src/chassis_controller.cpp
)
target_include_directories(case_study_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(case_study_controllers ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface case_study_plugins.xml)

install(TARGETS case_study_controllers DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config scripts DESTINATION share/${PROJECT_NAME})

ament_package()
