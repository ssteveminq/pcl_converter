#include "ros/ros.h"
#include "pcl_manager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_converter");

  Pcl_manager object_tracker;
  //object_tracker.handletarget_pub = object_tracker.n.advertise<sensor_msgs::PointCloud2>("/detected_final_handle_marker",50,true);
  
  ros::Rate loop(10);
  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();  
  }
  return 0;
}




