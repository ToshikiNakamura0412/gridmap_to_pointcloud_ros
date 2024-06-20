cmake_minimum_required(VERSION 3.8)
project(gridmap_to_pointcloud_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
)
find_package(PCL REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(gridmap_to_pointcloud_node src/gridmap_to_pointcloud.cpp)
target_link_libraries(gridmap_to_pointcloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
