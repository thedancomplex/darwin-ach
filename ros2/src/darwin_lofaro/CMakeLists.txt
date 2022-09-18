cmake_minimum_required(VERSION 3.8)
project(darwin_lofaro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra -Wpedantic -DLINUX -D_GNU_SOURCE -m32 -O2 -O3 -g )
  #add_compile_options(-Wall -Wextra -Wpedantic -DLINUX -D_GNU_SOURCE -m32 -O2 -O3 -g -lrt -ldxl_x86_cpp -lstdc++ -pthread)
endif()

set(CMAKE_C_FLAGS   "-Wall -DLINUX -D_GNU_SOURCE -Wall -m32 -O2 -O3 -g 
")
set(CMAKE_CXX_FLAGS "-Wall -DLINUX -D_GNU_SOURCE -Wall -m32 -O2 -O3 -g
")
#set(CMAKE_CXX_FLAGS "-Wall -DLINUX -D_GNU_SOURCE -Wall -m32 -O2 -O3 -g -lrt -ldxl_x86_cpp -lstdc++")


#file(GLOB SOURCES
#	dynamixel/include/*.h
#	dynamixel/include/dynamixel/*.h
#	dynamixel/include/dynamixel_sdk/*.h
#	dynamixel/src/*.h
#	dynamixel/src/dynamixel_sdk/*.cpp
#)


#set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

#set(THREADS_PREFER_PTHREADS_FLAG ON)
#set(CMAKE_THREAD_PREFER_PTHREADS TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

#set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
#set(THREADS_PREFER_PTHREAD_FLAG TRUE)
#find_package(Threads REQUIRED)

#set_property(TARGET ${PROJECT_NAME} PROPERTY IMPORTED_LOCATION "/home/robot/projects/DynamixelSDK/c++/build/linux32/*.so")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(${PROJECT_NAME} src/darwin_lofaro.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs)
target_include_directories(${PROJECT_NAME} PUBLIC
  /etc/darwin-legacy/include/
#  /etc/darwin-legacy/include/dynamixel/
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17




install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
