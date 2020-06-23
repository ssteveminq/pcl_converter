cmake_minimum_required(VERSION 2.8.3)
project(pcl_converter)	

SET(SOURCES src/pcl_manager)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport 
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
  image_geometry
  depth_image_proc
)

#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

#find_package(PCL 1.7 REQUIRED)

#add_service_files(
   #FILES
#)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   std_srvs
   sensor_msgs
)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  message_runtime
  tf
  image_geometry
  DEPENDS OpenCV
 )


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(pcl_test src/pcl_test.cpp ${SOURCES})
add_dependencies(pcl_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pcl_generator src/pcl_generator.cpp ${SOURCES})
add_dependencies(pcl_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_generator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#target_link_libraries(pcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
