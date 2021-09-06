/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * 
 * from https://docs.px4.io/master/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <takeoff.h>
#include <sensors.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle node;

    ros::AsyncSpinner spinner(8);
    spinner.start();

    Takeoff takeoff(node);
    takeoff.init();

    Sensors sensors(node);

    ros::Rate rate(20.0);

    ros::Time currtime {};
    double lasttime {0};
    double timediff {0};

    int cycle {0};
    geometry_msgs::Point desired_position;

    while (ros::ok() && cycle < 4)
    {
        //Return to (0,0,10)
        
        desired_position.x = 0;
        desired_position.y = 0;
        desired_position.z = 10;

        while(ros::ok() && timediff < 3)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sensors.get_position().x + 0.01 > desired_position.x && sensors.get_position().x - 0.01 < desired_position.x && 
                sensors.get_position().y + 0.01 > desired_position.y && sensors.get_position().y - 0.01 < desired_position.y &&
                sensors.get_position().z + 0.01 > desired_position.z && sensors.get_position().z - 0.01 < desired_position.z)
            {
                if (lasttime == 0){
                    lasttime = ros::Time::now().toSec();
                }
                currtime = ros::Time::now();
                timediff = currtime.toSec() - lasttime;
            }
        }

        
        timediff = 0;
        lasttime = 0;

        // Move foward 10 virtual meters.

        desired_position = sensors.get_position();
        desired_position.x += 10;

        while(ros::ok() && timediff < 3)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sensors.get_position().x + 0.01 > desired_position.x && sensors.get_position().x - 0.01 < desired_position.x && 
                sensors.get_position().y + 0.01 > desired_position.y && sensors.get_position().y - 0.01 < desired_position.y &&
                sensors.get_position().z + 0.01 > desired_position.z && sensors.get_position().z - 0.01 < desired_position.z)
            {
                if (lasttime == 0){
                    lasttime = ros::Time::now().toSec();
                }
                currtime = ros::Time::now();
                timediff = currtime.toSec() - lasttime;
            }
        }

        timediff = 0;
        lasttime = 0;

        // Move up (heave) 15 virtual meters.

        desired_position = sensors.get_position();
        desired_position.z += 15;

        while(ros::ok() && timediff < 3)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sensors.get_position().x + 0.01 > desired_position.x && sensors.get_position().x - 0.01 < desired_position.x && 
                sensors.get_position().y + 0.01 > desired_position.y && sensors.get_position().y - 0.01 < desired_position.y &&
                sensors.get_position().z + 0.01 > desired_position.z && sensors.get_position().z - 0.01 < desired_position.z)
            {
                if (lasttime == 0){
                    lasttime = ros::Time::now().toSec();
                }
                currtime = ros::Time::now();
                timediff = currtime.toSec() - lasttime;
            }
        }

        timediff = 0;
        lasttime = 0;

        // Move left 5 virtual meters.

        desired_position = sensors.get_position();
        desired_position.y += 5;

        while(ros::ok() && timediff < 3)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sensors.get_position().x + 0.01 > desired_position.x && sensors.get_position().x - 0.01 < desired_position.x && 
                sensors.get_position().y + 0.01 > desired_position.y && sensors.get_position().y - 0.01 < desired_position.y &&
                sensors.get_position().z + 0.01 > desired_position.z && sensors.get_position().z - 0.01 < desired_position.z)
            {
                if (lasttime == 0){
                    lasttime = ros::Time::now().toSec();
                }
                currtime = ros::Time::now();
                timediff = currtime.toSec() - lasttime;
            }
        }

        timediff = 0;
        lasttime = 0;

        cycle +=1;
    }

    //Return to (0,0,10)
        
    desired_position.x = 0;
    desired_position.y = 0;
    desired_position.z = 10;

    while(ros::ok() && timediff < 3)
    {
        takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
        if (sensors.get_position().x + 0.01 > desired_position.x && sensors.get_position().x - 0.01 < desired_position.x && 
            sensors.get_position().y + 0.01 > desired_position.y && sensors.get_position().y - 0.01 < desired_position.y &&
            sensors.get_position().z + 0.01 > desired_position.z && sensors.get_position().z - 0.01 < desired_position.z)
        {
            if (lasttime == 0){
                lasttime = ros::Time::now().toSec();
            }
            currtime = ros::Time::now();
            timediff = currtime.toSec() - lasttime;
        }
    }

    return 0;
}
