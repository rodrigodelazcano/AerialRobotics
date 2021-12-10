#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/PoseStamped.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
// started with this https://github.com/dominikbelter/opencv_example

void imuCallback(const geometry_msgs::PoseStampedConstPtr& imu_msg)
{
geometry_msgs::PoseStamped current_imu = *imu_msg;

  static tf2_ros::TransformBroadcaster br;
 
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "body_mavros";
  transformStamped.transform.translation.x = current_imu.pose.position.x;
  transformStamped.transform.translation.y = current_imu.pose.position.y;
  transformStamped.transform.translation.z = current_imu.pose.position.z;
  //tf2::Quaternion q;
  //q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = current_imu.pose.orientation.x;
  transformStamped.transform.rotation.y = current_imu.pose.orientation.y;
  transformStamped.transform.rotation.z = current_imu.pose.orientation.z;
  transformStamped.transform.rotation.w = current_imu.pose.orientation.w;

 //static tf2_ros::TransformBroadcaster br_world_flu;
  br.sendTransform(transformStamped);
  //transformStamped.header.stamp = ros::Time::now();
  //transformStamped.header.frame_id = "world";
  //transformStamped.child_frame_id = "world_FLU";
  //transformStamped.transform.translation.x = 0.0;
  //transformStamped.transform.translation.y = 0.0;
  //transformStamped.transform.translation.z = 0.0;
  //tf2::Quaternion q;
  //q.setRPY(0, 0, 1.57);
  //q.normalize();
  //transformStamped.transform.rotation.x = q[0];
  //transformStamped.transform.rotation.y = q[1];
  //transformStamped.transform.rotation.z = q[2];
  //transformStamped.transform.rotation.w = q[3];

  //br_world_flu.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "mavros_body_pos");

  // node handler
  ros::NodeHandle n;
  
  // subsribe topic
  ros::Subscriber imu_sub = n.subscribe("/mavros/local_position/pose", 1000, imuCallback);

  ros::spin();

  return 0;
}
