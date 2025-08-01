cmake_minimum_required(VERSION 3.8)
project(drone_nav_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp

      
)



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

install(
  DIRECTORY
    launch
    pcd
    srv
    config
    temp
  DESTINATION
    share/${PROJECT_NAME}/
)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Pos.srv"
  DEPENDENCIES geometry_msgs
  
)
ament_export_dependencies(rosidl_default_runtime)


add_executable(voxel src/voxel_filter.cpp)

ament_target_dependencies(voxel 
  rclcpp
  PCL
  # drone_nav_path_planner
  octomap
  visualization_msgs
)

target_link_libraries(voxel ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

install(TARGETS
  voxel
  DESTINATION lib/${PROJECT_NAME})

add_executable(local_voxel src/local_voxel_filter.cpp)

ament_target_dependencies(local_voxel 
  rclcpp
  PCL
  octomap
  visualization_msgs
  pcl_conversions
  pcl_msgs
  tf2_ros
  tf2
  tf2_eigen
)

target_link_libraries(local_voxel ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

install(TARGETS
  local_voxel
  DESTINATION lib/${PROJECT_NAME})

add_executable(global_voxel src/global_voxel_filter.cpp)

ament_target_dependencies(global_voxel 
  rclcpp
  PCL
  octomap
  visualization_msgs
  pcl_conversions
  pcl_msgs
  tf2_ros
  tf2
  tf2_eigen
)

target_link_libraries(global_voxel ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

install(TARGETS
  global_voxel
  DESTINATION lib/${PROJECT_NAME})

add_executable(path src/path.cpp)
target_link_libraries(path ${PROJECT_NAME}__rosidl_typesupport_cpp)
ament_target_dependencies(path 
  rclcpp
  nav_msgs
  PCL
  octomap
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

target_link_libraries(path ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

install(TARGETS
  path
  DESTINATION lib/${PROJECT_NAME})

# add_executable(bpath temp/bidirectional_path.cpp)

# ament_target_dependencies(bpath 
#   rclcpp
#   nav_msgs
#   PCL
#   octomap
#   visualization_msgs

# )

# target_link_libraries(bpath ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
# target_link_libraries(bpath ${PROJECT_NAME}__rosidl_typesupport_cpp)
# install(TARGETS
#   bpath
#   DESTINATION lib/${PROJECT_NAME})

add_executable(local_planner src/local_planner.cpp)

ament_target_dependencies(local_planner 
  rclcpp
  nav_msgs
  PCL
  octomap
  visualization_msgs

)

target_link_libraries(local_planner ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries(local_planner ${PROJECT_NAME}__rosidl_typesupport_cpp)
install(TARGETS
  local_planner
  DESTINATION lib/${PROJECT_NAME})



# install(PROGRAMS
#   temp/local_planner.py
#   temp/pcd_publisher.py
#   temp/path.py
#   DESTINATION lib/${PROJECT_NAME}
# )


ament_package()
