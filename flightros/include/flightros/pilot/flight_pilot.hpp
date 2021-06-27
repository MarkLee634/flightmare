
#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

// image
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

using namespace flightlib;

namespace flightros {

class FlightPilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FlightPilot();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void mainRenderCallback(const ros::TimerEvent& event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_1(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_2(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_3(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_4(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_5(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_6(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_7(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_8(const nav_msgs::Odometry::ConstPtr& msg);


  geometry_msgs::Point getPose_from_tf(const tf::StampedTransform& msg);
  geometry_msgs::Point project_2d_from_3d(const geometry_msgs::Point& msg);
  void init_camera_info();


  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher
  ros::Publisher camera_info_pub;
  ros::Publisher camera_info_pub_1;
  ros::Publisher camera_info_pub_2;
  ros::Publisher camera_info_pub_3;
  ros::Publisher camera_info_pub_4;
  ros::Publisher camera_info_pub_5;
  ros::Publisher camera_info_pub_6;
  ros::Publisher camera_info_pub_7;
  ros::Publisher camera_info_pub_8;

  // subscriber
  ros::Subscriber sub_state_est_;
  ros::Subscriber sub_state_est_1;
  ros::Subscriber sub_state_est_2;
  ros::Subscriber sub_state_est_3;
  ros::Subscriber sub_state_est_4;
  ros::Subscriber sub_state_est_5;
  ros::Subscriber sub_state_est_6;
  ros::Subscriber sub_state_est_7;
  ros::Subscriber sub_state_est_8;

  image_transport::Publisher rgb_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher rgb_pub_1_;
  image_transport::Publisher depth_pub_1_;
  image_transport::Publisher rgb_pub_2_;
  image_transport::Publisher depth_pub_2_;
  image_transport::Publisher rgb_pub_3_;
  image_transport::Publisher depth_pub_3_;
  image_transport::Publisher rgb_pub_4_;
  image_transport::Publisher depth_pub_4_;
  image_transport::Publisher rgb_pub_5_;
  image_transport::Publisher depth_pub_5_;
  image_transport::Publisher rgb_pub_6_;
  image_transport::Publisher depth_pub_6_;
  image_transport::Publisher rgb_pub_7_;
  image_transport::Publisher depth_pub_7_;
  image_transport::Publisher rgb_pub_8_;
  image_transport::Publisher depth_pub_8_;

  ros::Publisher track_bounding_box_pub_;
  image_transport::Publisher rgb_bounding_box_pub_;

  //camera info
  sensor_msgs::CameraInfo camera_info_msg;
  ros::Time camera_timestamp;

  //TF
  tf::TransformListener   tf_listener;
  tf::StampedTransform    tf_transform_relative_0_1;
  tf::StampedTransform    tf_transform_relative_0_2;
  tf::StampedTransform    tf_transform_relative_0_3;
  tf::StampedTransform    tf_transform_relative_0_4;
  tf::StampedTransform    tf_transform_relative_0_5;
  tf::StampedTransform    tf_transform_relative_0_6;
  tf::StampedTransform    tf_transform_relative_0_7;
  tf::StampedTransform    tf_transform_relative_0_8;

  //bounding box array
  geometry_msgs::PoseArray bbox_pose_array;



  // main image pub timer
  ros::Timer timer_main_loop_;
  // main render pub timer
  ros::Timer timer_render_loop_;

  // unity quadrotor1
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;

  // unity quadrotor1
  std::shared_ptr<Quadrotor> quad_ptr_1_;
  std::shared_ptr<RGBCamera> rgb_camera_1_;
  QuadState quad_state_1_;


  // unity quadrotor2
  std::shared_ptr<Quadrotor> quad_ptr_2_;
  std::shared_ptr<RGBCamera> rgb_camera_2_;
  QuadState quad_state_2_;

  // unity quadrotor3
  std::shared_ptr<Quadrotor> quad_ptr_3_;
  std::shared_ptr<RGBCamera> rgb_camera_3_;
  QuadState quad_state_3_;

  // unity quadrotor4
  std::shared_ptr<Quadrotor> quad_ptr_4_;
  std::shared_ptr<RGBCamera> rgb_camera_4_;
  QuadState quad_state_4_;

  // unity quadrotor5
  std::shared_ptr<Quadrotor> quad_ptr_5_;
  std::shared_ptr<RGBCamera> rgb_camera_5_;
  QuadState quad_state_5_;

  // unity quadrotor6
  std::shared_ptr<Quadrotor> quad_ptr_6_;
  std::shared_ptr<RGBCamera> rgb_camera_6_;
  QuadState quad_state_6_;

  // unity quadrotor7
  std::shared_ptr<Quadrotor> quad_ptr_7_;
  std::shared_ptr<RGBCamera> rgb_camera_7_;
  QuadState quad_state_7_;

  // unity quadrotor8
  std::shared_ptr<Quadrotor> quad_ptr_8_;
  std::shared_ptr<RGBCamera> rgb_camera_8_;
  QuadState quad_state_8_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliary variables
  Scalar main_loop_freq_{30.0};
  Scalar main_render_freq_{30.0};
};
}  // namespace flightros
