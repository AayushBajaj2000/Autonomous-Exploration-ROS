#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// Initial Coordinates of the robot based on the map coordinates
float x = 0; 
float y = 0;
int count = 0;

void moveToGoal(float x, float y){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
}

void moveToStart(const std_msgs::String::ConstPtr& msg)
{
  if(count == 0){
   std::cout<<"Im here"<<std::endl;
   moveToGoal(x,y);
   count++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_listener");
  // Getting the initial Coordinates of the robot
  nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
  // If we received a message then print it
  if (msg) {
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      std::cout<<x<<std::endl;
      std::cout<<y<<std::endl;
  }
  else std::cout<<"No message!"<<std::endl;

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("goalReached", 10, moveToStart);

  ros::spin();
}