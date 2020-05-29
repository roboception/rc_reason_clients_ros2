cmake_minimum_required(VERSION 3.5)

project(rc_reason_msgs)
include(cmake/project_version.cmake)

# - Configuration for code optimization -
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rc_common_msgs REQUIRED)

set(msg_files
  "msg/DetectedTag.msg"
  "msg/Tag.msg"
)

set(srv_files
  "srv/DetectTags.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs rc_common_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs rc_common_msgs)

ament_package()

######################################
## Define information for packaging ##
######################################
# optionally specify dependencies of the debian package here (comma separated!)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-$ENV{ROS_DISTRO}-std-msgs, ros-$ENV{ROS_DISTRO}-geometry-msgs, ros-$ENV{ROS_DISTRO}-rc-common-msgs")
include(cmake/package_debian.cmake)
