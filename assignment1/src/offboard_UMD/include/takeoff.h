#ifndef TAKEOFF_H
#define TAKEOFF_H

#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

class Takeoff
{
public:
    explicit Takeoff(ros::NodeHandle &node);

    void init();

    void current_state_callback(const mavros_msgs::State &msg);

    void go_to_position(const double x, const double y, const double z);

private:
    ros::NodeHandle node_;

    mavros_msgs::State current_state_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::Subscriber current_state_subscriber_;
    ros::Publisher local_pos_pub_;
};

#endif