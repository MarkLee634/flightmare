#include "flightros/pilot/flight_pilot.hpp"

#define use_multi true
#define CAMERA_RES_WIDTH 360 //180 //720
#define CAMERA_RES_HEIGHT 240 //120 //480
#define CAMERA_FOV 90
#define DRONE2CAM_OFFSET 0.5
namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(30.0),
    main_render_freq_(30.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad shared info
  Vector<3> B_r_BC(0, DRONE2CAM_OFFSET, 0);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;


  // ============================== drone 1 ==============================
  // quad initialization
  quad_ptr_1_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_1_ = std::make_shared<RGBCamera>();
  rgb_camera_1_->setFOV(CAMERA_FOV);
  rgb_camera_1_->setWidth(CAMERA_RES_WIDTH);
  rgb_camera_1_->setHeight(CAMERA_RES_HEIGHT);
  rgb_camera_1_->setRelPose(B_r_BC, R_BC);
  rgb_camera_1_->setPostProcesscing(
    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
  quad_ptr_1_->addRGBCamera(rgb_camera_1_);

  // initialization
  quad_state_1_.setZero();
  quad_ptr_1_->reset(quad_state_1_);


  // ============================== drone 2 ==============================
  // quad initialization
  quad_ptr_2_ = std::make_shared<Quadrotor>();

  // add mono camera
//  rgb_camera_2_ = std::make_shared<RGBCamera>();
//  rgb_camera_2_->setFOV(CAMERA_FOV);
//  rgb_camera_2_->setWidth(CAMERA_RES_WIDTH);
//  rgb_camera_2_->setHeight(CAMERA_RES_HEIGHT);
//  rgb_camera_2_->setRelPose(B_r_BC, R_BC);
//  rgb_camera_2_->setPostProcesscing(
//    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
//  quad_ptr_2_->addRGBCamera(rgb_camera_2_);

  // initialization
  quad_state_2_.setZero();
  quad_ptr_2_->reset(quad_state_2_);

  // ============================== drone 3 ==============================
  // quad initialization
  quad_ptr_3_ = std::make_shared<Quadrotor>();

  // add mono camera
//  rgb_camera_3_ = std::make_shared<RGBCamera>();
//  rgb_camera_3_->setFOV(CAMERA_FOV);
//  rgb_camera_3_->setWidth(CAMERA_RES_WIDTH);
//  rgb_camera_3_->setHeight(CAMERA_RES_HEIGHT);
//  rgb_camera_3_->setRelPose(B_r_BC, R_BC);
//  rgb_camera_3_->setPostProcesscing(
//    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
//  quad_ptr_3_->addRGBCamera(rgb_camera_3_);

  // initialization
  quad_state_3_.setZero();
  quad_ptr_3_->reset(quad_state_3_);

  // ============================== drone 4 ==============================
  // quad initialization
  quad_ptr_4_ = std::make_shared<Quadrotor>();

  // add mono camera
//  rgb_camera_4_ = std::make_shared<RGBCamera>();
//  rgb_camera_4_->setFOV(CAMERA_FOV);
//  rgb_camera_4_->setWidth(CAMERA_RES_WIDTH);
//  rgb_camera_4_->setHeight(CAMERA_RES_HEIGHT);
//  rgb_camera_4_->setRelPose(B_r_BC, R_BC);
//  rgb_camera_4_->setPostProcesscing(
//    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
//  quad_ptr_4_->addRGBCamera(rgb_camera_4_);

  // initialization
  quad_state_4_.setZero();
  quad_ptr_4_->reset(quad_state_4_);

  // ============================== drone 5 ==============================
  // quad initialization
  quad_ptr_5_ = std::make_shared<Quadrotor>();

  // add mono camera
//  rgb_camera_5_ = std::make_shared<RGBCamera>();
//  rgb_camera_5_->setFOV(CAMERA_FOV);
//  rgb_camera_5_->setWidth(CAMERA_RES_WIDTH);
//  rgb_camera_5_->setHeight(CAMERA_RES_HEIGHT);
//  rgb_camera_5_->setRelPose(B_r_BC, R_BC);
//  rgb_camera_5_->setPostProcesscing(
//    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
//  quad_ptr_5_->addRGBCamera(rgb_camera_5_);

  // initialization
  quad_state_5_.setZero();
  quad_ptr_5_->reset(quad_state_5_);

  // ============================== drone 6 ==============================
  // quad initialization
  quad_ptr_6_ = std::make_shared<Quadrotor>();

  // add mono camera
//  rgb_camera_6_ = std::make_shared<RGBCamera>();
//  rgb_camera_6_->setFOV(CAMERA_FOV);
//  rgb_camera_6_->setWidth(CAMERA_RES_WIDTH);
//  rgb_camera_6_->setHeight(CAMERA_RES_HEIGHT);
//  rgb_camera_6_->setRelPose(B_r_BC, R_BC);
//  rgb_camera_6_->setPostProcesscing(
//    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
//  quad_ptr_6_->addRGBCamera(rgb_camera_6_);

  // initialization
  quad_state_6_.setZero();
  quad_ptr_6_->reset(quad_state_6_);

  // ============================== drone 7 ==============================
  // quad initialization
  quad_ptr_7_ = std::make_shared<Quadrotor>();

  // add mono camera
//  rgb_camera_7_ = std::make_shared<RGBCamera>();
//  rgb_camera_7_->setFOV(CAMERA_FOV);
//  rgb_camera_7_->setWidth(CAMERA_RES_WIDTH);
//  rgb_camera_7_->setHeight(CAMERA_RES_HEIGHT);
//  rgb_camera_7_->setRelPose(B_r_BC, R_BC);
//  rgb_camera_7_->setPostProcesscing(
//    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
//  quad_ptr_7_->addRGBCamera(rgb_camera_7_);

  // initialization
  quad_state_7_.setZero();
  quad_ptr_7_->reset(quad_state_7_);

  // ============================== drone 8 ==============================
  // quad initialization
  quad_ptr_8_ = std::make_shared<Quadrotor>();

  // add mono camera
//  rgb_camera_8_ = std::make_shared<RGBCamera>();
//  rgb_camera_8_->setFOV(CAMERA_FOV);
//  rgb_camera_8_->setWidth(CAMERA_RES_WIDTH);
//  rgb_camera_8_->setHeight(CAMERA_RES_HEIGHT);
//  rgb_camera_8_->setRelPose(B_r_BC, R_BC);
//  rgb_camera_8_->setPostProcesscing(
//    std::vector<bool>{true, false, false});  // depth, segmentation, optical flow
//  quad_ptr_8_->addRGBCamera(rgb_camera_8_);

  // initialization
  quad_state_8_.setZero();
  quad_ptr_8_->reset(quad_state_8_);

  // ============================== drone 0 ==============================
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



  // ============================== subscribe and publish ==============================

  // initialize publisher
  image_transport::ImageTransport it(pnh);

  //publisher
  rgb_pub_ = it.advertise("/hummingbird0/camera/rgb",1);
  depth_pub_ = it.advertise("/hummingbird0/camera/depth",1);
  camera_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird0/camera/camera_info",1);

  //bounding box overlay RGB img
  rgb_bounding_box_pub_ = it.advertise("/hummingbird0/camera/bounding_box",1);

  //bounding box with 2D position
  track_bounding_box_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/hummingbird0/track/bounding_box",1);


//  rgb_pub_1_ = it.advertise("/hummingbird1/camera/rgb",1);
//  depth_pub_1_ = it.advertise("/hummingbird1/camera/depth",1);
//  camera_info_pub_1 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird1/camera/camera_info",1);

//  rgb_pub_2_ = it.advertise("/hummingbird2/camera/rgb",1);
//  depth_pub_2_ = it.advertise("/hummingbird2/camera/depth",1);
//  camera_info_pub_2 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird2/camera/camera_info",1);

//  rgb_pub_3_ = it.advertise("/hummingbird3/camera/rgb",1);
//  depth_pub_3_ = it.advertise("/hummingbird3/camera/depth",1);
//  camera_info_pub_3 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird3/camera/camera_info",1);

//  rgb_pub_4_ = it.advertise("/hummingbird4/camera/rgb",1);
//  depth_pub_4_ = it.advertise("/hummingbird4/camera/depth",1);
//  camera_info_pub_4 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird4/camera/camera_info",1);

//  rgb_pub_5_ = it.advertise("/hummingbird5/camera/rgb",1);
//  depth_pub_5_ = it.advertise("/hummingbird5/camera/depth",1);
//  camera_info_pub_5 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird5/camera/camera_info",1);

//  rgb_pub_6_ = it.advertise("/hummingbird6/camera/rgb",1);
//  depth_pub_6_ = it.advertise("/hummingbird6/camera/depth",1);
//  camera_info_pub_6 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird6/camera/camera_info",1);

//  rgb_pub_7_ = it.advertise("/hummingbird7/camera/rgb",1);
//  depth_pub_7_ = it.advertise("/hummingbird7/camera/depth",1);
//  camera_info_pub_7 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird7/camera/camera_info",1);

//  rgb_pub_8_ = it.advertise("/hummingbird8/camera/rgb",1);
//  depth_pub_8_ = it.advertise("/hummingbird8/camera/depth",1);
//  camera_info_pub_8 = nh_.advertise<sensor_msgs::CameraInfo>("/hummingbird8/camera/camera_info",1);


  init_camera_info();

  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate0", 1, &FlightPilot::poseCallback, this);

#ifdef use_multi
  sub_state_est_1 = nh_.subscribe("flight_pilot/state_estimate1", 1,&FlightPilot::poseCallback_1, this);
  sub_state_est_2 = nh_.subscribe("flight_pilot/state_estimate2", 1,&FlightPilot::poseCallback_2, this);
  sub_state_est_3 = nh_.subscribe("flight_pilot/state_estimate3", 1,&FlightPilot::poseCallback_3, this);
  sub_state_est_4 = nh_.subscribe("flight_pilot/state_estimate4", 1,&FlightPilot::poseCallback_4, this);
  sub_state_est_5 = nh_.subscribe("flight_pilot/state_estimate5", 1,&FlightPilot::poseCallback_5, this);
  sub_state_est_6 = nh_.subscribe("flight_pilot/state_estimate6", 1,&FlightPilot::poseCallback_6, this);
  sub_state_est_7 = nh_.subscribe("flight_pilot/state_estimate7", 1,&FlightPilot::poseCallback_7, this);
  sub_state_est_8 = nh_.subscribe("flight_pilot/state_estimate8", 1,&FlightPilot::poseCallback_8, this);

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

void FlightPilot::poseCallback_1(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_1_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_1_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_1_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_1_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_1_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_1_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_1_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_2(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_2_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_2_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_2_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_2_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_2_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_2_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_2_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_3(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_3_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_3_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_3_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_3_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_3_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_3_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_3_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_4(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_4_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_4_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_4_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_4_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_4_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_4_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_4_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_5(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_5_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_5_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_5_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_5_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_5_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_5_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_5_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_6(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_6_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_6_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_6_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_6_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_6_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_6_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_6_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_7(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_7_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_7_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_7_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_7_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_7_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_7_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_7_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::poseCallback_8(const nav_msgs::Odometry::ConstPtr &msg) {
#ifdef use_multi
  quad_state_8_.x[QS::POSX] = (Scalar)msg->pose.pose.position.y * -1;
  quad_state_8_.x[QS::POSY] = (Scalar)msg->pose.pose.position.x;
  quad_state_8_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_8_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_8_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.y * -1;
  quad_state_8_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_8_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  //rotate FlightPilot Render +90deg ZAxis to match Gazebo

#endif

}

void FlightPilot::mainRenderCallback(const ros::TimerEvent &event) {

#ifdef use_multi
  quad_ptr_1_->setState(quad_state_1_);
  quad_ptr_2_->setState(quad_state_2_);
  quad_ptr_3_->setState(quad_state_3_);
  quad_ptr_4_->setState(quad_state_4_);
  quad_ptr_5_->setState(quad_state_5_);
  quad_ptr_6_->setState(quad_state_6_);
  quad_ptr_7_->setState(quad_state_7_);
  quad_ptr_8_->setState(quad_state_8_);

#endif

  //Get next render with updated quad State
  quad_ptr_->setState(quad_state_);

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

  //add Image Data Retrieve
  cv::Mat img;
  cv::Mat img_depth;
  cv::Mat img_two;
  cv::Mat img_depth_two;
  cv::Mat img_three;
  cv::Mat img_four;
  cv::Mat img_depth_three;
  cv::Mat img_depth_four;
  cv::Mat img_bounding_box;

  camera_timestamp = ros::Time::now();

  //0th camera
  rgb_camera_->getRGBImage(img);
  sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  rgb_msg->header.stamp = camera_timestamp;
  rgb_pub_.publish(rgb_msg);

  rgb_camera_->getDepthMap(img_depth);
  sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_depth).toImageMsg();
  depth_msg->header.stamp = camera_timestamp;
  depth_pub_.publish(depth_msg);

//#ifdef use_multi
  // 1st camera
//  rgb_camera_1_->getRGBImage(img_two);
//  sensor_msgs::ImagePtr rgb_msg_two = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_two).toImageMsg();
//  rgb_msg_two->header.stamp = camera_timestamp;
//  rgb_pub_1_.publish(rgb_msg_two);


//  rgb_camera_1_->getDepthMap(img_two);
//  sensor_msgs::ImagePtr depth_msg_two = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_two).toImageMsg();
//  depth_msg_two->header.stamp = camera_timestamp;
//  depth_pub_1_.publish(depth_msg_two);

//  // 3rd camera
//  rgb_camera_3_->getRGBImage(img_three);
//  sensor_msgs::ImagePtr rgb_msg_three = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_three).toImageMsg();
//  rgb_msg_three->header.stamp = camera_timestamp;
//  rgb_pub_3_.publish(rgb_msg_three);


//  rgb_camera_3_->getDepthMap(img_depth_three);
//  sensor_msgs::ImagePtr depth_msg_three = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_depth_three).toImageMsg();
//  depth_msg_three->header.stamp = camera_timestamp;
//  depth_pub_3_.publish(depth_msg_three);

//  // 4th camera
//  rgb_camera_4_->getRGBImage(img_four);
//  sensor_msgs::ImagePtr rgb_msg_four = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_four).toImageMsg();
//  rgb_msg_four->header.stamp = camera_timestamp;
//  rgb_pub_4_.publish(rgb_msg_four);


//  rgb_camera_4_->getDepthMap(img_depth_four);
//  sensor_msgs::ImagePtr depth_msg_four = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_depth_four).toImageMsg();
//  depth_msg_four->header.stamp = camera_timestamp;
//  depth_pub_4_.publish(depth_msg_four);
//#endif

  //publish camera_Info
  camera_info_msg.header.stamp = camera_timestamp;
  camera_info_pub.publish(camera_info_msg);
  camera_info_pub_1.publish(camera_info_msg);
//  camera_info_pub_2.publish(camera_info_msg);
//  camera_info_pub_3.publish(camera_info_msg);
//  camera_info_pub_4.publish(camera_info_msg);

  //================ project 3D into 2D ================

  //lookup TF1,2

  try {
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird1/base_link",ros::Time(0), tf_transform_relative_0_1);
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird2/base_link",ros::Time(0), tf_transform_relative_0_2);
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird3/base_link",ros::Time(0), tf_transform_relative_0_3);
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird4/base_link",ros::Time(0), tf_transform_relative_0_4);
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird5/base_link",ros::Time(0), tf_transform_relative_0_5);
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird6/base_link",ros::Time(0), tf_transform_relative_0_6);
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird7/base_link",ros::Time(0), tf_transform_relative_0_7);
    tf_listener.lookupTransform("/hummingbird0/base_link_cam","/hummingbird8/base_link",ros::Time(0), tf_transform_relative_0_8);

  } catch (tf::TransformException ex){
    //ROS_WARN("%s",ex.what());
}

  //get T {x,y,z}
  geometry_msgs::Point transpose_0_1, transpose_0_2, transpose_0_3, transpose_0_4, transpose_0_5, transpose_0_6, transpose_0_7, transpose_0_8;
  transpose_0_1 = getPose_from_tf(tf_transform_relative_0_1);
  transpose_0_2 = getPose_from_tf(tf_transform_relative_0_2);
  transpose_0_3 = getPose_from_tf(tf_transform_relative_0_3);
  transpose_0_4 = getPose_from_tf(tf_transform_relative_0_4);
  transpose_0_5 = getPose_from_tf(tf_transform_relative_0_5);
  transpose_0_6 = getPose_from_tf(tf_transform_relative_0_6);
  transpose_0_7 = getPose_from_tf(tf_transform_relative_0_7);
  transpose_0_8 = getPose_from_tf(tf_transform_relative_0_8);

  //project 3D pose into 2D img coordinate
  geometry_msgs::Point projected_0_1, projected_0_2, projected_0_3, projected_0_4, projected_0_5, projected_0_6, projected_0_7, projected_0_8 ;
  projected_0_1 = project_2d_from_3d(transpose_0_1);
  projected_0_2 = project_2d_from_3d(transpose_0_2);
  projected_0_3 = project_2d_from_3d(transpose_0_3);
  projected_0_4 = project_2d_from_3d(transpose_0_4);
  projected_0_5 = project_2d_from_3d(transpose_0_5);
  projected_0_6 = project_2d_from_3d(transpose_0_6);
  projected_0_7 = project_2d_from_3d(transpose_0_7);
  projected_0_8 = project_2d_from_3d(transpose_0_8);

  int line_thickness = 2;
  int circle_radius = 4;//8


  //draw bounding box
  img_bounding_box = img.clone();


  //draw green circle
  cv::circle(img_bounding_box, cv::Point(projected_0_1.x,projected_0_1.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_0_2.x,projected_0_2.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_0_3.x,projected_0_3.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_0_4.x,projected_0_4.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_0_5.x,projected_0_5.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_0_6.x,projected_0_6.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_0_7.x,projected_0_7.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);
  cv::circle(img_bounding_box, cv::Point(projected_0_8.x,projected_0_8.y), circle_radius, cv::Scalar(0,255,0),line_thickness,8,0);

  //publish bounding box
  sensor_msgs::ImagePtr box_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_bounding_box).toImageMsg();
  box_msg->header.stamp = camera_timestamp;
  rgb_bounding_box_pub_.publish(box_msg);

  //empty previous frame data
  while (!bbox_pose_array.poses.empty())
     {
     bbox_pose_array.poses.pop_back();
     }

  //add px,py into vector
  geometry_msgs::Pose temp_pose;
  temp_pose.position.x = projected_0_1.x;
  temp_pose.position.y = projected_0_1.y;
  bbox_pose_array.poses.push_back(temp_pose);
  temp_pose.position.x = projected_0_2.x;
  temp_pose.position.y = projected_0_2.y;
  bbox_pose_array.poses.push_back(temp_pose);
  temp_pose.position.x = projected_0_3.x;
  temp_pose.position.y = projected_0_3.y;
  bbox_pose_array.poses.push_back(temp_pose);
  temp_pose.position.x = projected_0_4.x;
  temp_pose.position.y = projected_0_4.y;
  bbox_pose_array.poses.push_back(temp_pose);
  temp_pose.position.x = projected_0_5.x;
  temp_pose.position.y = projected_0_5.y;
  bbox_pose_array.poses.push_back(temp_pose);
  bbox_pose_array.header.stamp = camera_timestamp;
  temp_pose.position.x = projected_0_6.x;
  temp_pose.position.y = projected_0_6.y;
  bbox_pose_array.poses.push_back(temp_pose);
  temp_pose.position.x = projected_0_7.x;
  temp_pose.position.y = projected_0_7.y;
  bbox_pose_array.poses.push_back(temp_pose);
  temp_pose.position.x = projected_0_8.x;
  temp_pose.position.y = projected_0_8.y;

  bbox_pose_array.poses.push_back(temp_pose);
  //publish 2D track vector
  track_bounding_box_pub_.publish(bbox_pose_array);


}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();

#ifdef use_multi
    unity_bridge_ptr_->addQuadrotor(quad_ptr_1_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_2_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_3_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_4_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_5_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_6_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_7_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_8_);
#endif

    unity_bridge_ptr_->addQuadrotor(quad_ptr_);

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
