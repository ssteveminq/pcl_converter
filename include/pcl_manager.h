#include <vector>
#include <fstream>
#include <string>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class Pcl_manager{

public:
    Pcl_manager();
    ~Pcl_manager();

    ros::Publisher pcl_pub;
    ros::Subscriber rgb_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber point_sub;
    ros::NodeHandle n;
    tf::TransformListener 	  listener;
    void image_callback(const sensor_msgs::ImageConstPtr& input);
    void depthimage_callback(const sensor_msgs::ImageConstPtr& input);
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input);

};
