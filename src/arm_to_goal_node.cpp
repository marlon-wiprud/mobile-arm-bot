#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/JointTrajectoryAction.h>
// #include <arm_control.h>

geometry_msgs::Twist driveForward(geometry_msgs::Twist cmd)
{
  cmd.linear.x = 0.25;
  return cmd;
}

geometry_msgs::Twist turnLeft(geometry_msgs::Twist cmd)
{
  cmd.angular.z = 0.75;
  return cmd;
}

geometry_msgs::Twist turnRight(geometry_msgs::Twist cmd)
{ 
  cmd.angular.z = -0.75;
  return cmd;
}

geometry_msgs::Twist driveRight(geometry_msgs::Twist cmd)
{ 
  driveForward(cmd);
  turnRight(cmd);
  return cmd;
}

geometry_msgs::Twist driveLeft(geometry_msgs::Twist cmd)
{ 
  driveForward(cmd);
  turnLeft(cmd);
  return cmd;
}

ros::Publisher new_arm_publisher(ros::NodeHandle nh)
{
  return nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
}

ros::Publisher new_cmd_vel_publisher(ros::NodeHandle nh)
{
  return nh.advertise<geometry_msgs::Twist>("/robot_base_velocity_controller/cmd_vel", 1);
}

trajectory_msgs::JointTrajectory set_joint_position(trajectory_msgs::JointTrajectory traj, int idx, std::string name, int value)
{
  traj.joint_names[idx] = name;
  traj.points[0].positions[idx] = value;
  return traj;
}

trajectory_msgs::JointTrajectory new_arm_trajectory_action(int base_pos, int shoulder_pos, int bottom_wrist_pos, int elbow_pos, int top_wrist_pos)
{
  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.resize(5);
  traj.points.resize(1);
  traj.points[0].positions.resize(5);

  traj = set_joint_position(traj, 0, "arm_base_joint", base_pos);
  traj = set_joint_position(traj, 1, "shoulder_joint", shoulder_pos);
  traj = set_joint_position(traj, 2, "bottom_wrist_joint", bottom_wrist_pos);
  traj = set_joint_position(traj, 3, "elbow_joint", elbow_pos);
  traj = set_joint_position(traj, 4, "top_wrist_joint", top_wrist_pos);

  traj.points[0].time_from_start = ros::Duration(1);

  return traj;
}

void publish_arm_joint_trajectory(ros::Publisher arm_pub, trajectory_msgs::JointTrajectory traj)
{
  ros::Time beginTime = ros::Time::now();
  ros::Duration duration = ros::Duration(1); 
  ros::Time endTime = beginTime + duration;

  if(ros::ok())
  {
    ROS_INFO("publishing arm trajectory action...");
    while(ros::Time::now() < endTime)
    {
      arm_pub.publish(traj);
      ros::Duration(0.1).sleep(); 
    }

  }else{
    ROS_INFO("unable to publish arm trajectory action, ros not ok");
  } 
}

void publish_drive(ros::Publisher cmd_vel_pub, geometry_msgs::Twist(drive_func)(geometry_msgs::Twist msg))
{
  geometry_msgs::Twist drive_cmd;
  drive_cmd = drive_func(drive_cmd);
  cmd_vel_pub.publish(drive_cmd);
}

void keyboard_control(ros::NodeHandle nh, ros::Publisher arm_pub, ros::Publisher cmd_vel_pub) {

  // driving control
  const char key_forward = 'w';
  const char key_left = 'a';
  const char key_right = 'd';
  const char key_stop = 's';

  // arm control
  const char key_one = '1';
  const char key_zero = '0';
  const char key_shoulder_joint = 'i';

  char cmd[50];
  while(nh.ok())
  {
    std::cin.getline(cmd, 50);
    char key = cmd[0];

    switch (key)
    {
    case key_forward:
      publish_drive(cmd_vel_pub, driveForward);
      break;
    
    case key_left:
      publish_drive(cmd_vel_pub, driveLeft);
      break;
    
    case key_right:
      publish_drive(cmd_vel_pub, driveRight);
      break;

    case key_shoulder_joint:
      if(cmd[1] != key_zero && cmd[1] != key_one) {
        continue;
      }
      publish_arm_joint_trajectory(arm_pub, new_arm_trajectory_action(0, (int)cmd[1], 0, 0, 0));
      break;
    
    default:
      std::cout << "unknown command: " << cmd << "\n";
      break;
    }
  }

}

int main(int argc, char** argv)
{

  //   init the ROS node
  ros::init(argc, argv, "drive");
  ros::NodeHandle nh;

  // wait for valid simulated clock
  if(!ros::Time::waitForValid(ros::WallDuration(10.0)))
  {
    ROS_FATAL("timed out waiting for valid time");
    return EXIT_FAILURE;
  }

  ros::Publisher arm_pub = new_arm_publisher(nh);
  ros::Publisher cmd_vel_pub = new_cmd_vel_publisher(nh);

  keyboard_control(nh, arm_pub, cmd_vel_pub);
}