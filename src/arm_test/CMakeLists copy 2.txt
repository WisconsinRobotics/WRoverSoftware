cmake_minimum_required(VERSION 3.8)
project(arm_test_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)


add_executable(arm_test_node src/arm_test_node.cpp)
ament_target_dependencies(arm_test_node rclcpp std_msgs)

target_include_directories(arm_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR /home/wiscrobo/workspace/Phoenix5-Linux-Example/lib/arm32)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR /home/wiscrobo/workspace/Phoenix5-Linux-Example/lib/arm64)
else()
    set(PHOENIX_LIBS_DIR /home/wiscrobo/workspace/Phoenix5-Linux-Example/lib/x86-64)
endif()

install(TARGETS arm_test_node
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



include_directories(/home/wiscrobo/workspace/Phoenix5-Linux-Example/include)


target_link_libraries(arm_test_node ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(arm_test_node ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(arm_test_node ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixTools.so)

target_link_libraries(arm_test_node Threads::Threads)
target_link_libraries(arm_test_node ${SDL2_LIBRARIES})

add_custom_command(TARGET arm_test_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PHOENIX_LIBS_DIR}
        $<TARGET_FILE_DIR:arm_test_node>)
