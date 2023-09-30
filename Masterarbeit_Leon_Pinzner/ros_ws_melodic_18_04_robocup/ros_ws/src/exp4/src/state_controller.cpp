#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "std_msgs/String.h"
#include "rosgraph_msgs/Log.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RosoutReader
{
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_rout_;
  ros::Subscriber sub_pose_;
  ros::Publisher pub_goal_;

  
  bool exploration_stopped_ = false;

  geometry_msgs::PoseWithCovariance pose_robot_;
  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;

  int state_ = 0;

  float d_;
  float d_min = 0.05;

public:
  RosoutReader();
  ~RosoutReader();
  void RosoutCallback(const rosgraph_msgs::Log msg);
  void ReceiveRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_co);
};

RosoutReader::RosoutReader()
{
  sub_rout_ = n_.subscribe("/rosout",1,&RosoutReader::RosoutCallback,this);
  sub_pose_ = n_.subscribe("/base_footprint_pose",1,&RosoutReader::ReceiveRobotPoseCallback,this);
  pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,true);

  start.header.frame_id = "odom";
  start.pose.position.x = 0;
  start.pose.position.y = 0;
  start.pose.position.z = 0;

  goal.header.frame_id = "odom";
  //goal.pose.position.x = 1.6;
  //goal.pose.position.y = 4.12;
  goal.pose.position.x = 1.4;
  goal.pose.position.y = 1;
  goal.pose.position.z = 0;
}

RosoutReader::~RosoutReader()
{
}

void RosoutReader::RosoutCallback(const rosgraph_msgs::Log msg)
{
  if(state_ == 0){
    std::cout << "state 0\n";
    std::cout << msg.msg << "\n";
    if(msg.msg == "Exploration stopped."){
      std::cout << "state wechsel 0 -> 1\n";
      state_ = 1;
    }
  }
}

void RosoutReader::ReceiveRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_co)
{
  if(state_ == 1 || state_ == 2){

    pose_robot_= pose_co->pose;

    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal_mb;

    if(state_ == 1){
      std::cout << "state 1\n";

      goal_mb.target_pose.header.frame_id = goal.header.frame_id;
      goal_mb.target_pose.header.stamp = ros::Time::now();
      goal_mb.target_pose.pose.position.x = goal.pose.position.x;
      goal_mb.target_pose.pose.position.y = goal.pose.position.y;
      goal_mb.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal: \"goal\"");
      ac.sendGoal(goal_mb);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("goal reached");
        ROS_INFO("change state 1 -> 2");
        state_ = 2;
      }else{
        ROS_INFO("The base failed to move to \"goal\"");
        ROS_INFO("no state change, try again");
      }
    }

    if(state_ == 2){
      std::cout << "state 2\n";

      goal_mb.target_pose.header.frame_id = start.header.frame_id;
      goal_mb.target_pose.header.stamp = ros::Time::now();
      goal_mb.target_pose.pose.position.x = start.pose.position.x;
      goal_mb.target_pose.pose.position.y = start.pose.position.y;
      //goal_mb.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal: \"start\"");
      ac.sendGoal(goal_mb);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("start reached");
        ROS_INFO("change state 2 -> 1");
        state_ = 1;
      }else{
        ROS_INFO("The base failed to move to \"start\"");
        ROS_INFO("no state change, try again");
      }
    }

  }
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "state_controller");

  

  //Create an object of class SubscribeAndPublish that will take care of everything
  RosoutReader RosoutReaderObject;

  ros::spin();
  
  return 0;
}


//https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/?answer=59738?answer=59738#post-id-59738
