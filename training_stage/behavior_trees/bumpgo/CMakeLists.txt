cmake_minimum_required(VERSION 3.5)
project(bumpgo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# # Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/bumpgo/ForwardAction.cpp
  src/bumpgo/BackwardAction.cpp
  src/bumpgo/TurnAction.cpp
  src/bumpgo/ScanAction.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(bumpgo_main
  src/bumpgo.cpp
)
target_link_libraries(bumpgo_main ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees_xml)


install(TARGETS
  ${PROJECT_NAME}
  bumpgo_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
