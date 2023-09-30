#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <sstream>

using namespace grid_map;

class ModifyMap
{
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_map_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_pose_;

  GridMap map_received_;
  GridMap map_modified_;


  geometry_msgs::PoseWithCovariance pose_robot_;

  bool first_time_ = true;

  float ground_ = -0.195;

public:
  ModifyMap();
  ~ModifyMap();
  void ReceiveMapCallback(const grid_map_msgs::GridMap msg);
  void ReceiveRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_co);
};

ModifyMap::ModifyMap()
{
  pub_map_ = n_.advertise<grid_map_msgs::GridMap>("elevation_map_modified",1,true);
  sub_map_ = n_.subscribe("/elevation_mapping/elevation_map",1,&ModifyMap::ReceiveMapCallback,this);
  sub_pose_ = n_.subscribe("/base_footprint_pose",1,&ModifyMap::ReceiveRobotPoseCallback,this);

  map_received_.setFrameId("odom");
  map_modified_.setFrameId("odom");
}

ModifyMap::~ModifyMap()
{
}

void ModifyMap::ReceiveMapCallback(const grid_map_msgs::GridMap msg)
{
  // Receive GridMap
  GridMapRosConverter::fromMessage(msg,map_received_);
  ROS_INFO("received map with size %f x %f m (%i x %i cells).",
    map_received_.getLength().x(), map_received_.getLength().y(),
    map_received_.getSize()(0), map_received_.getSize()(1));

  // Configure the map using the received map data
  // This configuration occurs during the first callback
  if(first_time_){
    ROS_INFO("ReceiveMapCallback first time");
    
    map_modified_.setFrameId("odom");
    map_modified_.setGeometry(map_received_.getLength(),
                              map_received_.getResolution(),
                              map_received_.getPosition());

    ROS_INFO("set map_modified with size %f x %f m (%i x %i cells).",
      map_modified_.getLength().x(), map_modified_.getLength().y(),
      map_modified_.getSize()(0), map_modified_.getSize()(1));

    map_modified_.add("elevation");
    map_modified_.add("elevation_modified");
    map_modified_.add("elevation_fused");
    
    first_time_ = false;
  }
  
  // Copy the values of the received map
  for (GridMapIterator it(map_modified_); !it.isPastEnd(); ++it) {
    Position currentPosition;
    map_modified_.getPosition(*it, currentPosition); 
    map_modified_.atPosition("elevation",currentPosition) = map_received_.atPosition("elevation",currentPosition);
    map_modified_.atPosition("elevation_modified",currentPosition) = map_received_.atPosition("elevation",currentPosition);
  }

  // Determine the orientation of the robot
  float a_rad;
  a_rad = 2*acos(pose_robot_.pose.orientation.w);
  if(pose_robot_.pose.orientation.z < 0){          // Necessary because the angle might turn negative
    a_rad = a_rad*(-1);
  }

  float a_rad_factor = 0.5;
  float korrektur_rechts = 0;
  float korrektur_links = 0.1;
  float d_min = 0.55;
  float d_max = 2;

  // Calculation of a point d_min(m) away from the robot's orientation in the direction of orientation +- alpha
  float pos_x_plus = pose_robot_.pose.position.x + cos(a_rad+a_rad_factor+korrektur_rechts)*d_min;
  float pos_x_minus = pose_robot_.pose.position.x + cos(a_rad-a_rad_factor-korrektur_links)*d_min;
  float pos_y_plus = pose_robot_.pose.position.y + sin(a_rad+a_rad_factor+korrektur_rechts)*d_min;
  float pos_y_minus = pose_robot_.pose.position.y + sin(a_rad-a_rad_factor-korrektur_links)*d_min;
  Position pos_point1_x_y_plus(pos_x_plus,pos_y_plus);
  Position pos_point1_x_y_minus(pos_x_minus,pos_y_minus);


  // Calculation of a point d_max(m) away from the robot's orientation in the direction of orientation +- alpha
  pos_x_plus = pose_robot_.pose.position.x + cos(a_rad+a_rad_factor+korrektur_rechts)*d_max;
  pos_x_minus = pose_robot_.pose.position.x + cos(a_rad-a_rad_factor-korrektur_links)*d_max;
  pos_y_plus = pose_robot_.pose.position.y + sin(a_rad+a_rad_factor+korrektur_rechts)*d_max;
  pos_y_minus = pose_robot_.pose.position.y + sin(a_rad-a_rad_factor-korrektur_links)*d_max;
  Position pos_point2_x_y_plus(pos_x_plus,pos_y_plus);
  Position pos_point2_x_y_minus(pos_x_minus,pos_y_minus);

  // Check if the d_max points are outside the map + correction if yes
  if(!map_modified_.isInside(pos_point2_x_y_plus)){
    // ROS_INFO("point positve angle is outside the map");
    for(float i = d_max-0.1; i >= d_min; i = i-0.1){
      pos_x_plus = pose_robot_.pose.position.x + cos(a_rad+a_rad_factor)*i;
      pos_y_plus = pose_robot_.pose.position.y + sin(a_rad+a_rad_factor)*i;
      Position pos_point2_new_plus(pos_x_plus,pos_y_plus);
      if(map_modified_.isInside(pos_point2_new_plus)){
        pos_point2_x_y_plus = pos_point2_new_plus; 
        break;
      }
    }
  }

  if(!map_modified_.isInside(pos_point2_x_y_minus)){
    // ROS_INFO("point negative angle is outside the map");
    for(float i = d_max-0.1; i >= d_min; i = i-0.1){
      pos_x_minus = pose_robot_.pose.position.x + cos(a_rad-a_rad_factor)*i;
      pos_y_minus = pose_robot_.pose.position.y + sin(a_rad-a_rad_factor)*i;
      Position pos_point2_new_minus(pos_x_minus,pos_y_minus);
      if(map_modified_.isInside(pos_point2_new_minus)){
        pos_point2_x_y_minus = pos_point2_new_minus; //zeile glaub ich unnoetig
        break;
      }
    }
  }

  // erstellen des polygon ueber welches iteriert wird
  grid_map::Polygon polygon;
  polygon.setFrameId(map_modified_.getFrameId());
  polygon.addVertex(pos_point1_x_y_plus);
  polygon.addVertex(pos_point2_x_y_plus);
  polygon.addVertex(pos_point2_x_y_minus);
  polygon.addVertex(pos_point1_x_y_minus);
  polygon.addVertex(pos_point1_x_y_plus);

  // Highlighting the corners of the trapezoid on the map
  /* map_modified_.atPosition("elevation",pos_point1_x_y_plus) = 1;
  map_modified_.atPosition("elevation",pos_point2_x_y_plus) = 1;
  map_modified_.atPosition("elevation",pos_point2_x_y_minus) = 1;
  map_modified_.atPosition("elevation",pos_point1_x_y_minus) = 1;*/
  
  // Checking if the cell is NaN and if so, setting it to ground level
  if(map_modified_.isInside(pos_point1_x_y_plus) && map_modified_.isInside(pos_point2_x_y_plus) && 
      map_modified_.isInside(pos_point1_x_y_minus) && map_modified_.isInside(pos_point2_x_y_minus)){
    for (grid_map::PolygonIterator it(map_modified_, polygon);
        !it.isPastEnd(); ++it) {
      if(!( map_modified_.at("elevation", *it) > -5)){    
        map_modified_.at("elevation_modified", *it) = ground_;
      }
    }
  }

  // Iterating over the map, merging layers 'elevation' and 'ground_manipulated'
  for (GridMapIterator it(map_modified_); !it.isPastEnd(); ++it) {
    // If elevation > parcour -> elevation_fused = ground
    if( map_modified_.at("elevation_modified", *it)>0.1 ){
      map_modified_.at("elevation_fused", *it) = ground_;
    
    // If elevation != nan -> elevation_fused = elevation
    }else if(map_modified_.at("elevation", *it)>-5){
      map_modified_.at("elevation_fused", *it) = map_modified_.at("elevation", *it);
    
    // If elevation = nan und elevation_modified != nan -> elevation_fused = elevation_modified
    }else if( !(map_modified_.at("elevation", *it)>-5) && (map_modified_.at("elevation_modified", *it)>-5)){
      map_modified_.at("elevation_fused", *it) = map_modified_.at("elevation_modified", *it);
    }
  }

  grid_map_msgs::GridMap map_message;
  GridMapRosConverter::toMessage(map_modified_, map_message);
  pub_map_.publish(map_message);
}

// Receive the robot pose
void ModifyMap::ReceiveRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_co)
{
  pose_robot_= pose_co->pose;
}


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "modify_map_elevation");

  //Create an object of class SubscribeAndPublish that will take care of everything
  ModifyMap modify_map_object;

  ros::spin();
  
  return 0;
}
