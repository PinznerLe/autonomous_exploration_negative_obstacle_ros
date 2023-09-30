#include "ros/ros.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include <cmath>

using namespace grid_map;

GridMap map({"ele_act"});

ros::Publisher publisher;

tf2_ros::Buffer tfBuffer;

void callback_pc2(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ros::Time timeStamp = msg->header.stamp;
  map.setTimestamp(timeStamp.toNSec());

  sensor_msgs::PointCloud2 pc2_tf_odom;
  geometry_msgs::TransformStamped transformStamped;

  // Tf-transformation of the pointcloud
  try {
    transformStamped = tfBuffer.lookupTransform("odom", "camera_rgb_optical_frame", ros::Time(0));
    tf2::doTransform(*msg, pc2_tf_odom, transformStamped);
  } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
  }

  // Convert the pointcloud to pointcloudxyz
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pc2_tf_odom,*cloud);

  // Iterate through the cloud
  for(auto& point : cloud->points)
  {
    Position point_pos(point.x,point.y);
    if (map.isInside(point_pos)){
      map.atPosition("ele_act", point_pos) = point.z;
    }
  }
}
  




int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc2_to_gridmap");
  ros::NodeHandle n;
  publisher = n.advertise<grid_map_msgs::GridMap>("grid_map_pc", 1, true);
  ros::Subscriber sub_pc2 = n.subscribe("/camera/depth/points_downsampled", 1, callback_pc2);

  map.setFrameId("odom");
  
  map.setGeometry(Length(7.0, 10.0), 0.1, Position(0,0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

