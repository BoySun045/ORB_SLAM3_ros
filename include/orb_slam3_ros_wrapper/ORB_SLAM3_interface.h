#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "include/ImuTypes.h"
#include "include/System.h"
#include "orb_slam3_ros_wrapper/map_frame.h"

class ORB_SLAM3_interface
{
  ORB_SLAM3::System* mpSLAM;

  ros::NodeHandle* node_handle;

  ros::Publisher pose_pub;
  ros::Publisher map_frame_pub;
  ros::Publisher map_points_pub;
  image_transport::Publisher rendered_image_pub;

  std::string map_frame_id;
  std::string pose_frame_id;

public:
  ORB_SLAM3_interface(ORB_SLAM3::System* pSLAM, ros::NodeHandle* node_handle);

  // void rgb_callback(const sensor_msgs::ImageConstPtr& msgRGB);
  // void rgb_imu_callback(const sensor_msgs::ImageConstPtr& msgRGB);
  // void stereo_callback(const sensor_msgs::ImageConstPtr& msgRGB);
  void rgbd_callback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

  void publish_map_frame(cv::Mat Tcw, sensor_msgs::Image msgRGB, sensor_msgs::Image msgD,
                         ORB_SLAM3::System::eSensor sensor_type);
};