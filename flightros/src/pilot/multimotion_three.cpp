

#include <fstream>
#include <iostream>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>


ros::Publisher pub_arm_zero;
ros::Publisher pub_start_zero;
ros::Publisher pub_pose_zero;

ros::Publisher pub_arm_one;
ros::Publisher pub_start_one;
ros::Publisher pub_pose_one;

ros::Publisher pub_arm_two;
ros::Publisher pub_start_two;
ros::Publisher pub_pose_two;

ros::Publisher pub_arm_three;
ros::Publisher pub_start_three;
ros::Publisher pub_pose_three;

ros::Publisher pub_arm_four;
ros::Publisher pub_start_four;
ros::Publisher pub_pose_four;

ros::Publisher pub_arm_five;
ros::Publisher pub_start_five;
ros::Publisher pub_pose_five;

ros::Publisher pub_arm_six;
ros::Publisher pub_start_six;
ros::Publisher pub_pose_six;

ros::Publisher pub_arm_seven;
ros::Publisher pub_start_seven;
ros::Publisher pub_pose_seven;

ros::Publisher pub_arm_eight;
ros::Publisher pub_start_eight;
ros::Publisher pub_pose_eight;


void swapYZ(const geometry_msgs::PoseStamped inA, const geometry_msgs::PoseStamped inB)
{
  inA.pose.position.x;

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "multimotion");
  ros::NodeHandle nh;

  ros::Time startTime,endTime ,currentTime;

  ros::Rate loop_rate(10);


  //arm
  pub_arm_zero = nh.advertise<std_msgs::Bool>("/hummingbird0/bridge/arm", 1);
  pub_arm_one = nh.advertise<std_msgs::Bool>("/hummingbird1/bridge/arm", 1);
  pub_arm_two = nh.advertise<std_msgs::Bool>("/hummingbird2/bridge/arm", 1);
  pub_arm_three = nh.advertise<std_msgs::Bool>("/hummingbird3/bridge/arm", 1);
  pub_arm_four = nh.advertise<std_msgs::Bool>("/hummingbird4/bridge/arm", 1);
  pub_arm_five = nh.advertise<std_msgs::Bool>("/hummingbird5/bridge/arm", 1);
  pub_arm_six = nh.advertise<std_msgs::Bool>("/hummingbird6/bridge/arm", 1);
  pub_arm_seven = nh.advertise<std_msgs::Bool>("/hummingbird7/bridge/arm", 1);
  pub_arm_eight = nh.advertise<std_msgs::Bool>("/hummingbird8/bridge/arm", 1);

  //start
  pub_start_zero = nh.advertise<std_msgs::Empty>("/hummingbird0/autopilot/start", 1);
  pub_start_one = nh.advertise<std_msgs::Empty>("/hummingbird1/autopilot/start", 1);
  pub_start_two = nh.advertise<std_msgs::Empty>("/hummingbird2/autopilot/start", 1);
  pub_start_three = nh.advertise<std_msgs::Empty>("/hummingbird3/autopilot/start", 1);
  pub_start_four = nh.advertise<std_msgs::Empty>("/hummingbird4/autopilot/start", 1);
  pub_start_five = nh.advertise<std_msgs::Empty>("/hummingbird5/autopilot/start", 1);
  pub_start_six = nh.advertise<std_msgs::Empty>("/hummingbird6/autopilot/start", 1);
  pub_start_seven = nh.advertise<std_msgs::Empty>("/hummingbird7/autopilot/start", 1);
  pub_start_eight = nh.advertise<std_msgs::Empty>("/hummingbird8/autopilot/start", 1);


  //pose
  pub_pose_zero = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird0/autopilot/pose_command", 1);
  pub_pose_one = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird1/autopilot/pose_command", 1);
  pub_pose_two = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird2/autopilot/pose_command", 1);
  pub_pose_three = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird3/autopilot/pose_command", 1);
  pub_pose_four = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird4/autopilot/pose_command", 1);
  pub_pose_five = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird5/autopilot/pose_command", 1);
  pub_pose_six = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird6/autopilot/pose_command", 1);
  pub_pose_seven = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird7/autopilot/pose_command", 1);
  pub_pose_eight = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird8/autopilot/pose_command", 1);


  //data two arm, start
  std_msgs::Bool arm_zero, arm_one, arm_two, arm_three, arm_four, arm_five, arm_six, arm_seven, arm_eight;
  std_msgs::Empty start_zero, start_one, start_two, start_three, start_four, start_five, start_six, start_seven, start_eight;

  arm_zero.data = true;
  arm_one.data = true;
  arm_two.data = true;
  arm_three.data = true;
  arm_four.data = true;
  arm_five.data = true;
  arm_six.data = true;
  arm_seven.data = true;
  arm_eight.data = true;

  //=============================================

  geometry_msgs::PoseStamped pose_cmd_zero;
  pose_cmd_zero.pose.position.x = -3;
  pose_cmd_zero.pose.position.y = 0;
  pose_cmd_zero.pose.position.z = 3;

  geometry_msgs::PoseStamped pose_cmd_one;
  pose_cmd_one.pose.position.x = 0.5;
  pose_cmd_one.pose.position.y = -2;
  pose_cmd_one.pose.position.z = 3;

  geometry_msgs::PoseStamped pose_cmd_two;
  pose_cmd_two.pose.position.x = 1.5;
  pose_cmd_two.pose.position.y = 0;
  pose_cmd_two.pose.position.z = 5;

  geometry_msgs::PoseStamped pose_cmd_three;
  pose_cmd_three.pose.position.x = 2.5;
  pose_cmd_three.pose.position.y = 2;
  pose_cmd_three.pose.position.z = 3;


  //=============================================

  geometry_msgs::PoseStamped pose_cmd_one_next;
  pose_cmd_one_next.pose.position.x = pose_cmd_one.pose.position.x;
  pose_cmd_one_next.pose.position.y = pose_cmd_five.pose.position.y;
  pose_cmd_one_next.pose.position.z = pose_cmd_five.pose.position.z;

  geometry_msgs::PoseStamped pose_cmd_two_next;
  pose_cmd_two_next.pose.position.x = pose_cmd_two.pose.position.x;
  pose_cmd_two_next.pose.position.y = pose_cmd_six.pose.position.y;
  pose_cmd_two_next.pose.position.z = pose_cmd_six.pose.position.z;

  geometry_msgs::PoseStamped pose_cmd_three_next;
  pose_cmd_three_next.pose.position.x = pose_cmd_three.pose.position.x;
  pose_cmd_three_next.pose.position.y = pose_cmd_seven.pose.position.y;
  pose_cmd_three_next.pose.position.z = pose_cmd_seven.pose.position.z;

  geometry_msgs::PoseStamped pose_cmd_four_next;
  pose_cmd_four_next.pose.position.x = pose_cmd_four.pose.position.x;
  pose_cmd_four_next.pose.position.y = pose_cmd_eight.pose.position.y;
  pose_cmd_four_next.pose.position.z = pose_cmd_eight.pose.position.z;

  geometry_msgs::PoseStamped pose_cmd_five_next;
  pose_cmd_five_next.pose.position.x = pose_cmd_five.pose.position.x;
  pose_cmd_five_next.pose.position.y = pose_cmd_one.pose.position.y;
  pose_cmd_five_next.pose.position.z = pose_cmd_one.pose.position.z;

  geometry_msgs::PoseStamped pose_cmd_six_next;
  pose_cmd_six_next.pose.position.x = pose_cmd_six.pose.position.x;
  pose_cmd_six_next.pose.position.y = pose_cmd_two.pose.position.y;
  pose_cmd_six_next.pose.position.z = pose_cmd_two.pose.position.z;

  geometry_msgs::PoseStamped pose_cmd_seven_next;
  pose_cmd_seven_next.pose.position.x = pose_cmd_seven.pose.position.x;
  pose_cmd_seven_next.pose.position.y = pose_cmd_three.pose.position.y;
  pose_cmd_seven_next.pose.position.z = pose_cmd_three.pose.position.z;

  geometry_msgs::PoseStamped pose_cmd_eight_next;
  pose_cmd_eight_next.pose.position.x = pose_cmd_eight.pose.position.x;
  pose_cmd_eight_next.pose.position.y = pose_cmd_four.pose.position.y;
  pose_cmd_eight_next.pose.position.z = pose_cmd_four.pose.position.z;

  int stateMachine_counter = -1;

  startTime = ros::Time::now();

  int counter = 0;
#define movement_time 70

  int t1 = 10;
  int t2 = 30;
  int t3 = 80;
  int t4 = 115;
  int t5 = t4 + movement_time;
  int t6 = t5 + movement_time;
  int t7 = t6 + movement_time;






  while (ros::ok()) {

    counter++;
    //currentTime = ros::Time::now();
    //ROS_INFO("time past: %f\n",(currentTime - startTime).toSec());

    switch (stateMachine_counter)
    {

      case -1:
    {
      if (counter > t1)
      {
        ROS_INFO("publish arm");
        pub_arm_zero.publish(arm_zero);
        pub_arm_one.publish(arm_one);
        pub_arm_two.publish(arm_two);
        pub_arm_three.publish(arm_three);
        pub_arm_four.publish(arm_four);
        pub_arm_five.publish(arm_five);
        pub_arm_six.publish(arm_six);
        pub_arm_seven.publish(arm_seven);
        pub_arm_eight.publish(arm_eight);
        stateMachine_counter = 0;
      }

      break;
    }

    case 0:
    {
      if( counter > t2)
      {
        ROS_INFO("publish start");
        pub_start_zero.publish(start_zero);
        pub_start_one.publish(start_one);
        pub_start_two.publish(start_two);
        pub_start_three.publish(start_three);
        pub_start_four.publish(start_four);
        pub_start_five.publish(start_five);
        pub_start_six.publish(start_six);
        pub_start_seven.publish(start_seven);
        pub_start_eight.publish(start_eight);
        stateMachine_counter = 11;
      }
      break;
    }

    case 11:
    {
      if( counter > t3)
      {
        ROS_INFO("publish 0th pose");
        pub_pose_zero.publish(pose_cmd_zero);
        stateMachine_counter = 1;
      }
      break;
    }

    case 1:
    {

      if( counter > t4)
      {
        ROS_INFO("publish 1-8 pose");

        pub_pose_one.publish(pose_cmd_one);
        pub_pose_two.publish(pose_cmd_two);
        pub_pose_three.publish(pose_cmd_three);
        pub_pose_four.publish(pose_cmd_four);
        pub_pose_five.publish(pose_cmd_five);
        pub_pose_six.publish(pose_cmd_six);
        pub_pose_seven.publish(pose_cmd_seven);
        pub_pose_eight.publish(pose_cmd_eight);
        stateMachine_counter = 2;
      }
      break;
    }

    case 2:
    {
      if( counter > t5)
      {
        ROS_INFO("publish 1-8 pose2");
        pub_pose_one.publish(pose_cmd_one_next);
        pub_pose_two.publish(pose_cmd_two_next);
        pub_pose_three.publish(pose_cmd_three_next);
        pub_pose_four.publish(pose_cmd_four_next);
        pub_pose_five.publish(pose_cmd_five_next);
        pub_pose_six.publish(pose_cmd_six_next);
        pub_pose_seven.publish(pose_cmd_seven_next);
        pub_pose_eight.publish(pose_cmd_eight_next);

        stateMachine_counter = 3;
      }
      break;
    }

    case 3:
    {
      if( counter > t6)
      {
        ROS_INFO("publish 1-8 pose3");
        pub_pose_one.publish(pose_cmd_one);
        pub_pose_two.publish(pose_cmd_two);
        pub_pose_three.publish(pose_cmd_three);
        pub_pose_four.publish(pose_cmd_four);
        pub_pose_five.publish(pose_cmd_five);
        pub_pose_six.publish(pose_cmd_six);
        pub_pose_seven.publish(pose_cmd_seven);
        pub_pose_eight.publish(pose_cmd_eight);
        stateMachine_counter = 4;
      }
      break;
    }

    case 4:
    {
      if( counter > t7)
      {
        ROS_INFO("publish 1-8 pose4");

        pub_pose_one.publish(pose_cmd_one_next);
        pub_pose_two.publish(pose_cmd_two_next);
        pub_pose_three.publish(pose_cmd_three_next);
        pub_pose_four.publish(pose_cmd_four_next);
        pub_pose_five.publish(pose_cmd_five_next);
        pub_pose_six.publish(pose_cmd_six_next);
        pub_pose_seven.publish(pose_cmd_seven_next);
        pub_pose_eight.publish(pose_cmd_eight_next);

        stateMachine_counter = 5;
      }
      break;
    }

    case 5:
    {
      if( counter > 400)
      {
        ROS_INFO("end");
        return 0;
      }
      break;
    }




    } // end case


    ros::spinOnce();

    loop_rate.sleep();

  }



  return 0;
}
