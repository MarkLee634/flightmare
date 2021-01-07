#include "flightros/pilot/flight_pilot.hpp"

#define use_multi true
#define CAMERA_RES_WIDTH 180 //720
#define CAMERA_RES_HEIGHT 120 //480
#define CAMERA_FOV 90
namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(30.0),
    main_render_freq_(10.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad shared info
  Vector<3> B_r_BC(0, 0.3, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;

  // ============================== drone 1 ==============================
  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  rgb_camera_->setFOV(CAMERA_FOV);
  rgb_camera_->setWidth(CAMERA_RES_WIDTH);
  rgb_camera_->setHeight(CAMERA_RES_HEIGHT);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // ============================== drone 2 ==============================
  // quad initialization
  quad_ptr_two_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_two_ = std::make_shared<RGBCamera>();
  rgb_camera_two_->setFOV(CAMERA_FOV);
  rgb_camera_two_->setWidth(CAMERA_RES_WIDTH);
  rgb_camera_two_->setHeight(CAMERA_RES_HEIGHT);
  rgb_camera_two_->setRelPose(B_r_BC, R_BC);
  rgb_camera_two_->setPostProcesscing(
    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
  quad_ptr_two_->addRGBCamera(rgb_camera_two_);

  // initialization
  quad_state_two_.setZero();
  quad_ptr_two_->reset(quad_state_two_);

  // ============================== drone 3 ==============================
  // quad initialization
  quad_ptr_three_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_three_ = std::make_shared<RGBCamera>();
  rgb_camera_three_->setFOV(CAMERA_FOV);
  rgb_camera_three_->setWidth(CAMERA_RES_WIDTH);
  rgb_camera_three_->setHeight(CAMERA_RES_HEIGHT);
  rgb_camera_three_->setRelPose(B_r_BC, R_BC);
  rgb_camera_three_->setPostProcesscing(
    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
  quad_ptr_three_->addRGBCamera(rgb_camera_three_);

  // initialization
  quad_state_three_.setZero();
  quad_ptr_three_->reset(quad_state_three_);

  // ============================== subscribe and publish ==============================

  // initialize publisher
  image_transport::ImageTransport it(pnh);

  //publisher
  rgb_pub_ = it.advertise("/hummingbird1/camera/rgb",1);
  depth_pub_ = it.advertise("/hummingbird1/camera/depth",1);
  camera_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird1/camera/camera_info",1);

  rgb_pub_two_ = it.advertise("/hummingbird2/camera/rgb",1);
  depth_pub_two_ = it.advertise("/hummingbird2/camera/depth",1);
  camera_info_pub_two = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird2/camera/camera_info",1);

  rgb_pub_three_ = it.advertise("/hummingbird3/camera/rgb",1);
  depth_pub_three_ = it.advertise("/hummingbird3/camera/depth",1);
  camera_info_pub_three = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird3/camera/camera_info",1);

  //bounding box
  rgb_bounding_box_pub_ = it.advertise("/bounding_box",1);


  init_camera_info();

  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate1", 1, &FlightPilot::poseCallback, this);

#ifdef use_multi
  sub_state_est_two = nh_.subscribe("flight_pilot/state_estimate2", 1,&FlightPilot::poseCallback_two, this);
  sub_state_est_three = nh_.subscribe("flight_pilot/state_estimate3", 1,&FlightPilot::poseCallback_three, this);
#endif

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_), &FlightPilot::mainLoopCallback, this);
  timer_render_loop_ = nh_.createTimer(ros::Rate(main_render_freq_), &FlightPilot::mainRenderCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::init_camera_info() {
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]
  float f = ( float(CAMERA_RES_HEIGHT/2) / float(tan((M_PI*CAMERA_FOV/180)/2)) );
  float fx = f;
  float fy = f;
  float cx = CAMERA_RES_WIDTH/2;
  float cy = CAMERA_RES_HEIGHT/2;


  camera_info_msg.header.frame_id = "camera";
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.width = CAMERA_RES_WIDTH;
  camera_info_msg.height = CAMERA_RES_HEIGHT;
  camera_info_msg.K = {fx, 0, cx, 0, fy, cy, 0, 0, 1};

}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

}

void FlightPilot::poseCallback_two(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_two_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_two_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_two_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_two_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_two_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_two_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_two_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_three(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_three_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_three_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_three_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_three_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_three_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_three_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_three_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::mainRenderCallback(const ros::TimerEvent &event) {

  //Get next render with updated quad State
  quad_ptr_->setState(quad_state_);

#ifdef use_multi
  quad_ptr_two_->setState(quad_state_two_);
  quad_ptr_three_->setState(quad_state_three_);
#endif

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

  }
}

//return point from TF
geometry_msgs::Point FlightPilot::getPose_from_tf(const tf::StampedTransform &tf_msg) {

  geometry_msgs::Point temp_point;
  temp_point.x = tf_msg.getOrigin().x();
  temp_point.y = tf_msg.getOrigin().y();
  temp_point.z = tf_msg.getOrigin().z();
  //ROS_INFO("TF12 X: %f, Y: %f, Z: %f\n", temp_point.x , temp_point.y , temp_point.z );

  return temp_point;
}

//return 2D point from 3D projection
geometry_msgs::Point FlightPilot::project_2d_from_3d(const geometry_msgs::Point &point_msg) {

  geometry_msgs::Point projected_point;

  //default init
  projected_point.x = CAMERA_RES_WIDTH/2;
  projected_point.y = CAMERA_RES_HEIGHT/2;

  //project 3D to 2D
  //px = (-1) *fy*y/x + cx (row)
  //py = (-1) *fx*z/x + cy (col)
  projected_point.x = -1*camera_info_msg.K[4]*point_msg.y/point_msg.x + camera_info_msg.K[2];
  projected_point.y = -1*camera_info_msg.K[0]*point_msg.z/point_msg.x + camera_info_msg.K[5];

  //ROS_INFO("px12 pX: %d, pY: %d\n", (int)projected_point.x, (int)projected_point.y);

  return projected_point;
}


void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty

  //add Image Data Retrieve
  cv::Mat img;
  cv::Mat img_depth;
  cv::Mat img_two;
  cv::Mat img_depth_two;
  cv::Mat img_three;
  cv::Mat img_depth_three;
  cv::Mat img_bounding_box;

  camera_timestamp = ros::Time::now();

  //1st camera
  rgb_camera_->getRGBImage(img);
  sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp = camera_timestamp;
  rgb_pub_.publish(rgb_msg);

  rgb_camera_->getDepthMap(img_depth);
  sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_depth).toImageMsg();
  depth_msg->header.stamp = camera_timestamp;
  depth_pub_.publish(depth_msg);

#ifdef use_multi
  // 2nd camera
  rgb_camera_two_->getRGBImage(img_two);
  sensor_msgs::ImagePtr rgb_msg_two = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_two).toImageMsg();
  rgb_msg_two->header.stamp = camera_timestamp;
  rgb_pub_two_.publish(rgb_msg_two);


  rgb_camera_two_->getDepthMap(img_depth_two);
  sensor_msgs::ImagePtr depth_msg_two = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_depth_two).toImageMsg();
  depth_msg_two->header.stamp = camera_timestamp;
  depth_pub_two_.publish(depth_msg_two);

  // 3rd camera
  rgb_camera_three_->getRGBImage(img_three);
  sensor_msgs::ImagePtr rgb_msg_three = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_three).toImageMsg();
  rgb_msg_three->header.stamp = camera_timestamp;
  rgb_pub_three_.publish(rgb_msg_three);


  rgb_camera_three_->getDepthMap(img_depth_three);
  sensor_msgs::ImagePtr depth_msg_three = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_depth_three).toImageMsg();
  depth_msg_three->header.stamp = camera_timestamp;
  depth_pub_three_.publish(depth_msg_three);
#endif

  //publish camera_Info
  camera_info_msg.header.stamp = camera_timestamp;
  camera_info_pub.publish(camera_info_msg);
  camera_info_pub_two.publish(camera_info_msg);
  camera_info_pub_three.publish(camera_info_msg);

  //================ project 3D into 2D ================

  //lookup TF1,2

  try {
    tf_listener.lookupTransform("/hummingbird1/base_link","/hummingbird2/base_link",ros::Time(0), tf_transform_relative_one_two);
    tf_listener.lookupTransform("/hummingbird1/base_link","/hummingbird3/base_link",ros::Time(0), tf_transform_relative_one_three);

  } catch (tf::TransformException ex){
    ROS_WARN("%s",ex.what());
}

  //get T {x,y,z}
  geometry_msgs::Point transpose_one_two, transpose_one_three;
  transpose_one_two = getPose_from_tf(tf_transform_relative_one_two);
  transpose_one_three = getPose_from_tf(tf_transform_relative_one_three);

  //project 3D pose into 2D img coordinate
  geometry_msgs::Point projected_one_two, projected_one_three;
  projected_one_two = project_2d_from_3d(transpose_one_two);
  projected_one_three = project_2d_from_3d(transpose_one_three);



  int pt_offset_y = +5;
  int line_thickness = 2;


  //draw bounding box
  img_bounding_box = img.clone();

  /*
  //scale bounding box size
  float xmin = 1, xmax = 6, scalemin = 0.05, scalemax = 0.5;
  float scale = scalemax - ( ((x - xmin)/(xmax-xmin))*(scalemax-scalemin) );
  float width_scale = scale*CAMERA_RES_WIDTH;
  float height_scale = scale*CAMERA_RES_HEIGHT;
  ROS_INFO("scale: %f, scaleW: %f, scaleH: %f\n", scale, width_scale/2, height_scale/2);
  //draw green bounding box
  cv::rectangle(img_bounding_box, cv::Point(px-width_scale/2, py-height_scale/2), cv::Point(px+width_scale/2, py+height_scale), cv::Scalar(0,255,0), line_thickness, 8,0);
*/

  //draw green circle
  cv::circle(img_bounding_box, cv::Point(projected_one_two.x,projected_one_two.y+pt_offset_y), 4, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_one_three.x,projected_one_three.y+pt_offset_y), 4, cv::Scalar(0,255,0),line_thickness,8,0);

  //publish bounding box
  sensor_msgs::ImagePtr box_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_bounding_box).toImageMsg();
  box_msg->header.stamp = camera_timestamp;
  rgb_bounding_box_pub_.publish(box_msg);






}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
#ifdef use_multi
    unity_bridge_ptr_->addQuadrotor(quad_ptr_two_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_three_);
#endif
    ROS_WARN("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros
