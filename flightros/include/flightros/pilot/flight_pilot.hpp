
#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


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
  void poseCallback_two(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback_three(const nav_msgs::Odometry::ConstPtr& msg);
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
  ros::Publisher camera_info_pub_two;
  ros::Publisher camera_info_pub_three;

  // subscriber
  ros::Subscriber sub_state_est_;
  ros::Subscriber sub_state_est_two;
  ros::Subscriber sub_state_est_three;

  image_transport::Publisher rgb_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher rgb_pub_two_;
  image_transport::Publisher depth_pub_two_;
  image_transport::Publisher rgb_pub_three_;
  image_transport::Publisher depth_pub_three_;
  image_transport::Publisher rgb_bounding_box_pub_;

  //camera info
  sensor_msgs::CameraInfo camera_info_msg;
  ros::Time camera_timestamp;

  //TF
  tf::TransformListener   tf_listener;
  tf::StampedTransform    tf_transform_relative_one_two;
  tf::StampedTransform    tf_transform_relative_one_three;



  // main image pub timer
  ros::Timer timer_main_loop_;
  // main render pub timer
  ros::Timer timer_render_loop_;

  // unity quadrotor1
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;

  // unity quadrotor2
  std::shared_ptr<Quadrotor> quad_ptr_two_;
  std::shared_ptr<RGBCamera> rgb_camera_two_;
  QuadState quad_state_two_;

  // unity quadrotor3
  std::shared_ptr<Quadrotor> quad_ptr_three_;
  std::shared_ptr<RGBCamera> rgb_camera_three_;
  QuadState quad_state_three_;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliary variables
  Scalar main_loop_freq_{30.0};
  Scalar main_render_freq_{10.0};
};
}  // namespace flightros
