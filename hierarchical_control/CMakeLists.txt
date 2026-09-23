cmake_minimum_required(VERSION 3.16)
project(hierarchical_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  lifecycle_msgs
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_execution_group test/test_execution_group.cpp)
  target_include_directories(test_execution_group PRIVATE include)
  ament_target_dependencies(test_execution_group ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_stale_state_cost test/test_stale_state_cost.cpp)
  target_include_directories(test_stale_state_cost PRIVATE include)
  ament_target_dependencies(test_stale_state_cost ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_scheduling_performance test/test_scheduling_performance.cpp)
  target_include_directories(test_scheduling_performance PRIVATE include)
  ament_target_dependencies(test_scheduling_performance ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_pass_lower_bound test/test_pass_lower_bound.cpp)
  target_include_directories(test_pass_lower_bound PRIVATE include)
  ament_target_dependencies(test_pass_lower_bound ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_static_topology test/test_static_topology.cpp)
  target_include_directories(test_static_topology PRIVATE include)
  ament_target_dependencies(test_static_topology ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_dimensional_interfaces test/test_dimensional_interfaces.cpp)
  target_include_directories(test_dimensional_interfaces PRIVATE include)
  ament_target_dependencies(test_dimensional_interfaces ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_topology_contract test/test_topology_contract.cpp)
  target_include_directories(test_topology_contract PRIVATE include)
  ament_target_dependencies(test_topology_contract ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_topology_binding test/test_topology_binding.cpp)
  target_include_directories(test_topology_binding PRIVATE include)
  ament_target_dependencies(test_topology_binding ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_typed_ports test/test_typed_ports.cpp)
  target_include_directories(test_typed_ports PRIVATE include)
  ament_target_dependencies(test_typed_ports ${THIS_PACKAGE_INCLUDE_DEPENDS})

  # The cycle guarantee can only be tested by compiling files that MUST FAIL, so it runs as a
  # script rather than a gtest. CXX is forwarded so the corpus uses the same compiler as the build.
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_test(
      NAME test_static_topology_negative
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_static_topology_negative.py
    )
    set_tests_properties(
      test_static_topology_negative PROPERTIES ENVIRONMENT "CXX=${CMAKE_CXX_COMPILER}"
    )
  endif()
endif()

ament_package()
