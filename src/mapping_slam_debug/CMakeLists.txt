cmake_minimum_required(VERSION 3.8)
project(mapping_slam_debug)

find_package(ament_cmake REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

add_library(logging_solver SHARED src/logging_solver.cpp)
target_compile_features(logging_solver PUBLIC cxx_std_17)
target_include_directories(logging_solver PRIVATE ${slam_toolbox_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(logging_solver ${slam_toolbox_LIBRARIES} Boost::serialization)
ament_target_dependencies(logging_solver pluginlib rclcpp)

pluginlib_export_plugin_description_file(slam_toolbox solver_plugins.xml)

install(TARGETS logging_solver LIBRARY DESTINATION lib)

ament_package()
