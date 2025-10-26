cmake_minimum_required(VERSION 3.8)
project(arm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH "/usr/lib/phoenix6")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(SDL2 REQUIRED)

# --- Phoenix 6 manual integration ---
include_directories(/usr/include/phoenix6)
link_directories(/usr/lib/phoenix6)

# --- Add executables ---
add_executable(ik_test_node src/ik_test_node.cpp)
add_executable(arm_test_node src/arm_test_node.cpp)
add_executable(arm_test_shoulder src/arm_test_shoulder.cpp)
#add_executable(arm_orchestra_node src/arm_orchestra_node.cpp)

# --- ROS 2 dependencies ---
ament_target_dependencies(ik_test_node rclcpp std_msgs)
ament_target_dependencies(arm_test_node rclcpp std_msgs)
#ament_target_dependencies(arm_orchestra_node rclcpp std_msgs)
ament_target_dependencies(arm_test_shoulder rclcpp std_msgs)

# --- Include directories ---
target_include_directories(ik_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(arm_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(arm_test_shoulder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
#target_include_directories(arm_orchestra_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

# --- Compiler features ---
target_compile_features(ik_test_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(arm_test_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(arm_test_shoulder PUBLIC c_std_99 cxx_std_17)
#target_compile_features(arm_orchestra_node PUBLIC c_std_99 cxx_std_17)

# --- Link libraries (fixed) ---
target_link_libraries(ik_test_node
    CTRE_Phoenix6
    CTRE_PhoenixTools
    Threads::Threads
    ${SDL2_LIBRARIES}
)

target_link_libraries(arm_test_node
    CTRE_Phoenix6
    CTRE_PhoenixTools
    Threads::Threads
    ${SDL2_LIBRARIES}
)


target_link_libraries(arm_test_shoulder
    CTRE_Phoenix6
    CTRE_PhoenixTools
    Threads::Threads
    ${SDL2_LIBRARIES}
)



#target_link_libraries(arm_orchestra_node CTRE_Phoenix6 Threads::Threads ${SDL2_LIBRARIES})

# --- Install ---
install(TARGETS ik_test_node arm_test_node arm_test_shoulder #arm_orchestra_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

