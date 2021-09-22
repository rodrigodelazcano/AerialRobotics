/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.019.x, PX4 Pro Flight
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
#include <offboard/Cycle.h>
#include <math.h> 

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

    ros::Publisher cycle_pub = node.advertise<offboard::Cycle>("mavros/path_cycle", 1);

    ros::Rate rate(20.0);

    ros::Time currtime {};
    double lasttime {0};
    double timediff {0};

    int cycle {0};
    offboard::Cycle cyc;
    geometry_msgs::Point desired_position;

    while (ros::ok() && cycle < 4)
    {
        //Return to (0,0,10)
        
        desired_position.x = 0;
        desired_position.y = 0;
        desired_position.z = 10;

        // stay for 2 seconds
        while(ros::ok() && timediff < 2)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sensors.get_position().x + 0.05 > desired_position.x && sensors.get_position().x - 0.05 < desired_position.x && 
                sensors.get_position().y + 0.05 > desired_position.y && sensors.get_position().y - 0.05 < desired_position.y &&
                sensors.get_position().z + 0.05 > desired_position.z && sensors.get_position().z - 0.05 < desired_position.z)
            {
                if (lasttime == 0){
                    lasttime = ros::Time::now().toSec();
                }
                currtime = ros::Time::now();
                timediff = currtime.toSec() - lasttime;
            }
        }

        // 1 cycle has been performed
        cyc.stamp.sec = ros::Time::now().toSec();
        cyc.stamp.nsec = ros::Time::now().toNSec();
        cyc.cycle = cycle;
        cycle_pub.publish(cyc);

        
        timediff = 0;
        lasttime = 0;

        // Move foward 10 virtual meters.

        desired_position = sensors.get_position();
        desired_position.x += 10;

        // stay for 2 seconds
        while(ros::ok() && timediff < 2)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sqrt(pow(sensors.get_position().x-desired_position.x, 2) + 
                pow(sensors.get_position().y-desired_position.y, 2) + 
                pow(sensors.get_position().z-desired_position.z, 2)) < 0.05)
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

        // stay for 2 seconds
        while(ros::ok() && timediff < 2)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sqrt(pow(sensors.get_position().x-desired_position.x, 2) + 
                pow(sensors.get_position().y-desired_position.y, 2) + 
                pow(sensors.get_position().z-desired_position.z, 2)) < 0.05)
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

        // stay for 5 seconds
        while(ros::ok() && timediff < 5)
        {
            takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
            if (sqrt(pow(sensors.get_position().x-desired_position.x, 2) + 
                pow(sensors.get_position().y-desired_position.y, 2) + 
                pow(sensors.get_position().z-desired_position.z, 2)) < 0.01)
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

    // stay for 2 seconds
    while(ros::ok() && timediff < 2)
    {
        takeoff.go_to_position(desired_position.x, desired_position.y, desired_position.z);
        if (sqrt(pow(sensors.get_position().x-desired_position.x, 2) + 
                pow(sensors.get_position().y-desired_position.y, 2) + 
                pow(sensors.get_position().z-desired_position.z, 2)) < 0.05)
        {
            if (lasttime == 0){
                lasttime = ros::Time::now().toSec();
            }
            currtime = ros::Time::now();
            timediff = currtime.toSec() - lasttime;
        }
    }

    cyc.stamp.sec = ros::Time::now().toSec();
    cyc.stamp.nsec = ros::Time::now().toNSec();
    cyc.cycle = cycle;
    cycle_pub.publish(cyc);

    return 0;
}
