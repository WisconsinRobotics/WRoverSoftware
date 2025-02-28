cmake_minimum_required(VERSION 3.8)
project(arm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Og")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(SDL2 REQUIRED)
find_package(phoenix6 REQUIRED) # Ensure this is the correct package name

add_executable(ik_test_node src/ik_test_node.cpp)
ament_target_dependencies(ik_test_node rclcpp std_msgs)

add_executable(arm_test_node src/ik_test_node.cpp)
ament_target_dependencies(arm_test_node rclcpp std_msgs)

target_include_directories(ik_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(arm_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(ik_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(arm_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link against the correct libraries
target_link_libraries(ik_test_node phoenix6 Threads::Threads ${SDL2_LIBRARIES})
target_link_libraries(arm_test_node phoenix6 Threads::Threads ${SDL2_LIBRARIES})

install(TARGETS ik_test_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS arm_test_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
