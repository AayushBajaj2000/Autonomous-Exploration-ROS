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

// Send the action to the move_base node to move the robot to the start
void moveToGoal(float x, float y){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //Set the goal to be the initial position of the robot
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1;

  // Send the goal
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait forr the results
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, auto exploration is done!!!");
  else
    ROS_INFO("The robot could not move back to the start!!");
}

// Function which moves the robot to the start
void moveToStart(const std_msgs::String::ConstPtr& msg)
{
  // Just do it once in case we receiver multiple messages (usually we get 2)
  if(count == 0){
   std::cout<<"Going back to the Start!!"<<std::endl;
   moveToGoal(x,y);
   count++;
  }
}

int main(int argc, char **argv)
{
  // initializing the listener node
  ros::init(argc, argv, "odom_listener");
  
  // Getting the initial Coordinates of the robot from odom topic
  nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
  
  // If we received a message then print it and save the coordinates
  if (msg) {
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      std::cout<<x<<std::endl;
      std::cout<<y<<std::endl;
  }
  else std::cout<<"No message!"<<std::endl;

  ros::NodeHandle n;
  // Subscibe to the goalReached topic which tells us when the exploration is done.
  ros::Subscriber sub = n.subscribe("goalReached", 1 , moveToStart);

  ros::spin();
}