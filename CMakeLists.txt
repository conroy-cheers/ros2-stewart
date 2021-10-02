cmake_minimum_required(VERSION 3.5)
project(stewart)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# FetchContent added in CMake 3.11, downloads during the configure step
include(FetchContent)
fetchcontent_declare(
  hexapod_kinematics
  GIT_REPOSITORY https://github.com/conroy-cheers/hexapod_kinematics.git
  GIT_TAG master)
fetchcontent_makeavailable(hexapod_kinematics)
find_package(hexapod_kinematics REQUIRED)

add_executable(stewart_kinematics_node src/stewart_kinematics_node.cpp)
target_include_directories(
  stewart_kinematics_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    hexapod_kinematics)
target_link_libraries(stewart_kinematics_node hexapod_kinematics)
ament_target_dependencies(stewart_kinematics_node hexapod_kinematics rclcpp std_msgs geometry_msgs std_srvs)

install(TARGETS stewart_kinematics_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
