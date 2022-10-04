#include "orb_slam3_ros_wrapper/ORB_SLAM3_interface.h"
#include "sophus/geometry.hpp"

#include <vector>

ORB_SLAM3_interface::ORB_SLAM3_interface(ORB_SLAM3::System* pSLAM, ros::NodeHandle* node_handle)
  : mpSLAM(pSLAM), node_handle(node_handle)
{
  frame_pub = node_handle->advertise<orb_slam3_ros_wrapper::frame>("/frames", 1);
  pose_pub = node_handle->advertise<geometry_msgs::PoseStamped>("/orb/cam_pose",1);

  std::string node_name = ros::this_node::getName();
  node_handle->param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
  node_handle->param<std::string>(node_name + "/pose_frame_id", pose_frame_id, "pose");

  prev_sample_time = ros::Time::now();
}

void ORB_SLAM3_interface::rgbd_callback(const sensor_msgs::ImageConstPtr& msgRGB,
                                        const sensor_msgs::ImageConstPtr& msgD)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try
  {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Main algorithm runs here
  Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

  publish_frame(Tcw, *msgRGB, *msgD, ORB_SLAM3::System::STEREO);
  //  publish_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), cv_ptrRGB->header.stamp);
}

geometry_msgs::PoseStamped ORB_SLAM3_interface::SE3toPoseMsg(Sophus::SE3f tf)
{
  Eigen::Isometry3d camera_tf;
  camera_tf.matrix() = tf.matrix().cast<double>();

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = map_frame_id;

  pose_msg.pose.position.x = camera_tf.translation().x();
  pose_msg.pose.position.y = camera_tf.translation().y();
  pose_msg.pose.position.z = camera_tf.translation().z();

  Eigen::Quaterniond q(camera_tf.linear());
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  return pose_msg;
}

void ORB_SLAM3_interface::publish_frame(Sophus::SE3f Tcw, sensor_msgs::Image msgRGB, sensor_msgs::Image msgD,
                                            ORB_SLAM3::System::eSensor sensor_type)
{
  orb_slam3_ros_wrapper::frame frame_msg;
  frame_msg.rgb = msgRGB;
  frame_msg.depth = msgD;
  frame_msg.pose = SE3toPoseMsg(Tcw).pose;

  geometry_msgs::PoseStamped cam_pose;
  // cam_pose.header.stamp = ros::Time::now();
  cam_pose.header.stamp = msgRGB.header.stamp;
  cam_pose.header.frame_id = map_frame_id;

  cam_pose.pose.position.x = frame_msg.pose.position.x;
  cam_pose.pose.position.y = frame_msg.pose.position.y;
  cam_pose.pose.position.z = frame_msg.pose.position.z;

  cam_pose.pose.orientation.x = frame_msg.pose.orientation.x;
  cam_pose.pose.orientation.y = frame_msg.pose.orientation.y;
  cam_pose.pose.orientation.z = frame_msg.pose.orientation.z;
  cam_pose.pose.orientation.w = frame_msg.pose.orientation.w;

  pose_pub.publish(cam_pose);

  frame_pub.publish(frame_msg);

  publish_tf(cam_pose);

}

void ORB_SLAM3_interface::publish_tf(geometry_msgs::PoseStamped cam_pose)
{
    // publish a static transform between cam init and world ref frame
  
  if(cam_pose_init == false)
  {
    tf::TransformListener *tf_ = new tf::TransformListener;
    tf::StampedTransform transform_odom;
    ros::Time t = ros::Time(0);
    tf_->waitForTransform("multijackal_01/odom", "multijackal_01/d435i_color_optical_frame", t, ros::Duration(2.0));
    tf_->lookupTransform("multijackal_01/odom", "multijackal_01/d435i_color_optical_frame", t, transform_odom);
    x = transform_odom.getOrigin().getX();
    y = transform_odom.getOrigin().getY();
    z = transform_odom.getOrigin().getZ();

    q_odom.setX(transform_odom.getRotation().getX());
    q_odom.setY(transform_odom.getRotation().getY());
    q_odom.setZ(transform_odom.getRotation().getZ());
    q_odom.setW(transform_odom.getRotation().getW());
    
    cam_pose_init = true; 
    cam_initpose_pub = true;
    std::cout << "found the initial camera position" << std::endl;
    std::cout << "it is pos : " << x << "," << y << "," << z << "," << std::endl;
    std::cout << "and rot: " << q_odom.x() << "," << q_odom.y() << "," << q_odom.z() << "," << q_odom.w() << std::endl;
  }

  if(cam_initpose_pub == true)
  {
    tf::Transform transform_;
    transform_.setOrigin( tf::Vector3(x, y, z));
    transform_.setRotation(q_odom);
    br.sendTransform(tf::StampedTransform(transform_, cam_pose.header.stamp, "multijackal_01/odom", map_frame_id)); 
  }

  tf::Transform transform;
  tf::Quaternion q;
  q.setX(cam_pose.pose.orientation.x);
  q.setY(cam_pose.pose.orientation.y);
  q.setZ(cam_pose.pose.orientation.z);
  q.setW(cam_pose.pose.orientation.w);

  transform.setRotation(q);
  transform.setOrigin( tf::Vector3(cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z) );
  br.sendTransform(tf::StampedTransform(transform.inverse(), cam_pose.header.stamp,map_frame_id,pose_frame_id)); 

}