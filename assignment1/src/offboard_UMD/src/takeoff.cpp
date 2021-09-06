#include "takeoff.h"

Takeoff::Takeoff(ros::NodeHandle &node)
{
    node_ = node;

    arming_client_ =
        node_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    set_mode_client_ =
        node_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    current_state_subscriber_ = 
        node_.subscribe(
            "mavros/state", 10,
             &Takeoff::current_state_callback, this
        );

    local_pos_pub_ = 
        node_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
}

void Takeoff::init()
{
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    std::string mode {};
    // wait for FCU connection
    while(ros::ok() && !current_state_.connected){
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 10;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub_.publish(pose);
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    while(ros::ok() && current_state_.mode != "OFFBOARD")
    {
       if( set_mode_client_.call(offb_set_mode) &&
               offb_set_mode.response.mode_sent){
               ROS_INFO("Offboard enabled");
           }       
        rate.sleep();
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while(ros::ok() && !current_state_.armed)
    {
       if( arming_client_.call(arm_cmd) &&
                   arm_cmd.response.success){
                   ROS_INFO("Vehicle armed");
               }
        rate.sleep();
    }

}

void Takeoff::current_state_callback(const mavros_msgs::State &msg)
{
    current_state_ = msg;
}

void Takeoff::go_to_position(const double x, const double y, const double z)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    local_pos_pub_.publish(pose);   
}