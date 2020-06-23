#include "pcl_manager.h"
#include <string>
#include <iostream>
#include <cmath>


Pcl_manager::~Pcl_manager(){}
Pcl_manager::Pcl_manager()
{
    //rgb_sub = n.subscribe<sensor_msgs::Image>("/frontleft_rgb_image", 
            //10, &Pcl_manager::image_callback,this);
    //depth_sub =n.subscribe<sensor_msgs::Image>("frontleft_depth_image", 
            //10, &Pcl_manager::depthimage_callback,this);

    point_sub =n.subscribe<sensor_msgs::PointCloud2>("point_cloud/points", 
            10, &Pcl_manager::pcl_callback,this);
}

void Pcl_manager::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{

}

void Pcl_manager::image_callback(const sensor_msgs::ImageConstPtr& input)
{
    ROS_INFO("rgb");
}

void Pcl_manager::depthimage_callback(const sensor_msgs::ImageConstPtr& input)
{
    ROS_INFO("depth");
}

