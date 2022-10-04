cmake_minimum_required(VERSION 3.5)
project(ros2_ach_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


#add_compile_options("-lach")
#set(CMAKE_C_FLAGS   -lach)
#set(CMAKE_CXX_FLAGS -lach)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic -lach)
  #add_compile_options(-Wall -Wextra -Wpedantic -DLINUX -D_GNU_SOURCE -m32 -O2 -O3 -g)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find dependencies
find_package(ament_cmake REQUIRED)
#find_package(dynamixel_sdk REQUIRED)
#find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

# Build
add_executable(${PROJECT_NAME} src/ros2_ach_bridge.cpp)
add_executable(${PROJECT_NAME}_imu src/ros2_ach_bridge_imu.cpp)

#set_target_properties(${PROJECT_NAME} PROPERTIES IMPORTED_LOCATION /usr/lib/libach.so /usr/lib/libach.so.4 /usr/lib/libach.so.4.0.0)

target_link_libraries(${PROJECT_NAME} ach)
target_link_libraries(${PROJECT_NAME}_imu ach)

# Dependencies
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  std_msgs 
  geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME}_imu
  rclcpp 
  std_msgs 
  geometry_msgs
)

# Directeries
target_include_directories(${PROJECT_NAME} PUBLIC
  /etc/darwin-legacy/include/
  include/
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}_imu PUBLIC
  /etc/darwin-legacy/include/
  include/
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
)


# Install
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_imu
  DESTINATION lib/${PROJECT_NAME}_imu
)

# Test
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
