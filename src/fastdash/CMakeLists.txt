cmake_minimum_required(VERSION 3.8)
project(fastdash)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the path to the pigpio includes.
find_path(pigpio_INCLUDE_DIR 
	NAMES pigpio.h pigpiod_if.h pigpiod_if2.h
	HINTS /usr/local/include)
	
# Find the pigpio libraries.
find_library(pigpio_LIBRARY 
	NAMES libpigpio.so
	HINTS /usr/local/lib)
find_library(pigpiod_if_LIBRARY 
	NAMES libpigpiod_if.so
	HINTS /usr/local/lib)
find_library(pigpiod_if2_LIBRARY 
	NAMES libpigpiod_if2.so
	HINTS /usr/local/lib)
    
# Set the pigpio variables to plural form to make them accessible for 
# the paramount cmake modules.
set(pigpio_INCLUDE_DIRS ${pigpio_INCLUDE_DIR})
set(pigpio_INCLUDES     ${pigpio_INCLUDE_DIR})

# Handle REQUIRED, QUIET, and version arguments 
# and set the <packagename>_FOUND variable.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pigpio 
    DEFAULT_MSG 
    pigpio_INCLUDE_DIR pigpio_LIBRARY pigpiod_if_LIBRARY pigpiod_if2_LIBRARY)



# FetchContent_Declare(
#   caplin
#   GIT_REPOSITORY https://github.com/pragmaticlinuxblog/caplin
#   GIT_TAG        1.0.0 
# )
# FetchContent_MakeAvailable(caplin)

# FetchContent_GetProperties(caplin)
#     if(NOT caplin_POPULATED)
#       message("Cloning caplin")
#       FetchContent_Populate(caplin)
#       add_subdirectory(
#         ${caplin_SOURCE_DIR}
#         ${caplin_BINARY_DIR})
#     endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(dash_msgs REQUIRED)
# find_package(libpigpio REQUIRED)


# find_package()

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

add_executable(fastdash include/fastdash.hpp src/fastdash.cpp)

TARGET_LINK_LIBRARIES(fastdash dl -lboost_system)# -pthread  -lcrypt -lm -lpthread)

ament_target_dependencies(fastdash rclcpp rosbag2_cpp std_msgs can_msgs dash_msgs example_interfaces pigpio)


install(TARGETS
fastdash
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
