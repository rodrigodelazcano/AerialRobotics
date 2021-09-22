#include "sensors.h"

Sensors::Sensors(ros::NodeHandle &node)
{
    node_ = node;

    position_subscriber_ = 
        node_.subscribe(
            "mavros/local_position/pose", 10,
             &Sensors::position_callback, this
        );
}

void Sensors::position_callback(const geometry_msgs::PoseStamped &msg)
{
    position_ = msg.pose.position;
}