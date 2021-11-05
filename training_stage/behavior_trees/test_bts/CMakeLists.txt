cmake_minimum_required(VERSION 3.5)
project(test_bts)

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
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/test_bts/vacuum_cleaner/CheckBattery.cpp
  src/test_bts/vacuum_cleaner/CleanAction.cpp
  src/test_bts/vacuum_cleaner/Recharge.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(vacuum_cleaner
  src/vacuum_cleaner.cpp
)
target_link_libraries(vacuum_cleaner ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bts_xml)


install(TARGETS
  ${PROJECT_NAME}
  vacuum_cleaner
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

