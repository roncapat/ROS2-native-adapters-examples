cmake_minimum_required(VERSION 3.5)
project(cloud_to_depth)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_common REQUIRED)
find_package(native_adapters REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(pcl_conversions REQUIRED)

include_directories(include)

add_library(CloudToDepth SHARED src/CloudToDepth.cpp)
ament_target_dependencies(CloudToDepth SYSTEM rclcpp rcutils rclcpp_components OpenCV sensor_msgs native_adapters image_common cv_bridge pcl_conversions)
target_link_libraries(CloudToDepth ${PCL_LIBRARIES})

rclcpp_components_register_node(
  CloudToDepth 
  PLUGIN "perception::CloudToDepth"
  EXECUTABLE CloudToDepthExecutable)

install(TARGETS CloudToDepth
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
