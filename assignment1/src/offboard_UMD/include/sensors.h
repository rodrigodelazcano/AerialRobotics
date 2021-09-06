#ifndef SENSORS_H
#define SENSORS_H

#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

class Sensors
{
public:
    explicit Sensors(ros::NodeHandle &node);

    void init();

    void position_callback(const geometry_msgs::PoseStamped &msg);

    geometry_msgs::Point get_position()
    {
        return position_;
    }

private:
    ros::NodeHandle node_;

    ros::Subscriber position_subscriber_;

    geometry_msgs::Point position_;
};

#endif