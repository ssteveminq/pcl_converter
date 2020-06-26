#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h> 

// Approximate time policy
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
using namespace message_filters;

// Darknet detection
// PCL specific includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <depth_image_proc/depth_traits.h>

#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/common/centroid.h>

namespace enc = sensor_msgs::image_encodings;
// Initialize publishers
ros::Publisher pub_door;
ros::Publisher pub_cloud;

// Initialize transform listener
tf::TransformListener* lst;

// Set fixed reference frame
std::string fixed_frame = "odom";
sensor_msgs::CameraInfo Info;

void get_CameraInfo(sensor_msgs::CameraInfo& c_info )
{
   //sensor_msgs::CameraInfo c_info;
   c_info.header.frame_id="odom";
   c_info.height=720;
   c_info.width=1280;
   c_info.distortion_model= "plumb_bob";
   //c_info.D[0]=0.0;
   //c_info.D[1]=0.0;
   //c_info.D[2]=0.0;
   //c_info.D[3]=0.0;
   //c_info.D[4]=0.0;
   //c_info.D=[0.0, 0.0, 0.0, 0.0, 0.0]
   c_info.K[0]=925.13330078125;
   c_info.K[1]=0.0;
   c_info.K[2]=634.3031616210938;
   c_info.K[3]=0.0;
   c_info.K[4]=925.2440185546875;
   c_info.K[5]=366.37713623046875;
   c_info.K[6]=0.0;
   c_info.K[7]=0.0;
   c_info.K[8]=1.0;
   //c_info.K=[925.13330078125, 0.0,634.3031616210938,  0.0, 925.2440185546875, 366.37713623046875,  0.0, 0.0, 1.0]
   c_info.R[0]=1.0;     c_info.R[1]=0.0; c_info.R[2]=0.0;
   c_info.R[3]=0.0;     c_info.R[4]=1.0; c_info.R[5]=0.0;
   c_info.R[6]=0.0;     c_info.R[7]=0.0; c_info.R[8]=1.0;

   c_info.P[0]=925.13330078125;
   c_info.P[1]=0.0;
   c_info.P[2]=634.3031616210938;
   c_info.P[3]=0.0;
   c_info.P[4]=0.0;
   c_info.P[5]=925.2440185546875;
   c_info.P[6]=366.37713623046875;
   c_info.P[7]=0.0;
   c_info.P[8]=0.0;
   c_info.P[9]=0.0;
   c_info.P[10]=1.0;
   c_info.P[11]=0.0;
   //c_info.P=[925.13330078125, 0.0,634.3031616210938,  0.0, 0.0, 925.2440185546875, 366.37713623046875,  0.0, 0.0, 0.0, 1.0, 0.0]
   c_info.binning_x=0;
   c_info.binning_y=0;
   ROS_INFO("camera paramters intialized");

}



template<typename T>
void convert(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg,
                                      const sensor_msgs::PointCloud2::Ptr& cloud_msg,
                                      int red_offset, int green_offset, int blue_offset, int color_step)
{

  image_geometry::PinholeCameraModel model_;
  model_.fromCameraInfo(Info);
  // Use correct principal point from calibration
  float center_x = model_.cx();
  float center_y = model_.cy();
  ROS_INFO("center_x", center_x);
  ROS_INFO("center_y", center_y);

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model_.fx();
  float constant_y = unit_scaling / model_.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

  for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
  {
    for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
    {
      T depth = depth_row[u];

      // Check for invalid measurements
      if (!depth_image_proc::DepthTraits<T>::valid(depth))
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
      else
      {
        // Fill in XYZ
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);
      }

      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}



void 
image_cb (const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{

  // Initialize containers for clouds
  ROS_INFO("image callback");
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud                  (new pcl::PointCloud<pcl::PointXYZRGB>);
  //
  if (depth_msg->header.frame_id != rgb_msg->header.frame_id)
  {
    ROS_INFO("Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                           depth_msg->header.frame_id.c_str(), rgb_msg->header.frame_id.c_str());
    return;
  }

  if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
  {
      ROS_INFO("Images should be resized");
  
  }
// Supported color encodings: RGB8, BGR8, MONO8
  int red_offset, green_offset, blue_offset, color_step;
  if (rgb_msg->encoding == enc::RGB8)
  {
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::BGR8)
  {
    red_offset   = 2;
    green_offset = 1;
    blue_offset  = 0;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::MONO8)
  {
    red_offset   = 0;
    green_offset = 0;
    blue_offset  = 0;
    color_step   = 1;
  }
  else
  {
    try
    {
        //ROS_INFO("FIXME");
      //rgb_msg = (cv_bridge::toCvCopy(rgb_msg, enc::RGB8)->toImageMsg());
       cv_bridge::toCvCopy(rgb_msg, enc::RGB8)->toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_INFO("Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
      return;
    }
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }

// Allocate new point cloud message
  sensor_msgs::PointCloud2::Ptr cloud_msg (new sensor_msgs::PointCloud2);
  cloud_msg->header = depth_msg->header; // Use depth image time stamp
  cloud_msg->header.frame_id="odom";
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  if (depth_msg->encoding == enc::TYPE_16UC1)
  {
    ROS_INFO("16UC1");
    convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  }
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    ROS_INFO("32FC1");
    convert<float>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  }
  else
  {
    ROS_INFO("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_cloud.publish (cloud_msg);



  // Create a container for the result data.
  //sensor_msgs::PointCloud2 output_door;
    // Convert pcl::PointCloud to sensor_msgs::PointCloud2
  //pcl::toROSMsg(*cloud_door,output_door);
    // Set output frame as the input frame
  //output_door.header.frame_id             = input_cloud->header.frame_id;
  // Publish the data.
  //pub_door.publish (output_door);

  ROS_INFO("Pointcloud processed");
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_processing");
  ros::NodeHandle nh;

  get_CameraInfo(Info);

  // Initialize subscribers to darknet detection and pointcloud
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb(nh, "rgb/image", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh, "depth/image", 1);
  //message_filters::Subscribe<sensor_msgs::Cameramage> sub_depth(nh, "depth_registered/image_rect", 1);

  // Initialize transform listener
  tf::TransformListener listener(ros::Duration(10));
  lst = &listener;

  // Synchronize darknet detection with pointcloud
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_rgb, sub_depth);
  sync.registerCallback(boost::bind(&image_cb, _1, _2));

  // Create a ROS publisher for the output point cloud
  pub_cloud= nh.advertise<sensor_msgs::PointCloud2> ("output_points", 1);

  // Spin
  ros::spin ();
}
