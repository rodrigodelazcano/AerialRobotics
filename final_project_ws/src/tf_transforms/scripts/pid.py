#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, Int64
import tf

def pos_reference_cb(msg):
	global pos_reference_msg
	pos_reference_msg = msg

def stage_idx_cb(msg):
	global stage_idx
	stage_idx = msg.data

KP = [1.5,  1.5, 1.5,  0.5]
KD = [0.0, 0.0, 0.0, 0.0]
KI = [0.0, 0.0, 0.01, 0.0]

PI = 3.14159265359

vel_cmd_msg = TwistStamped()

def constraint(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def main():

    rospy.init_node("pid_node")  
    rospy.Subscriber("/drone/reference_goal", PoseStamped, pos_reference_cb)
    rospy.Subscriber("/drone/stage_idx", Int64, stage_idx_cb)
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    global pos_reference_msg, stage_idx
    stage_idx = Int64()
    pos_reference_msg = PoseStamped()
    listener = tf.TransformListener()

    # Integral cumulative error
    Iterm = [0, 0, 0, 0]
    error = [0, 0, 0, 0]
    V_PID = [0, 0, 0, 0]
    prev_error = [0, 0, 0, 0]#prev error vaal
    v_pid = [0.0,0.0,0.0, 0]#process varible declare
    vel_max = 1.5
    yaw_rate_max = 0.4
    I_max = 1.0

    rate = rospy.Rate(20.0)
    sample_time = 1/20.0

    while not rospy.is_shutdown():
        
        if stage_idx == 2 or stage_idx == 3 or stage_idx == 4:
            (trans_drone, rot_drone) = listener.lookupTransform('/world', '/body_mavros', rospy.Time(0))

            orientation_list_drone = [rot_drone[0], rot_drone[1], rot_drone[2], rot_drone[3]]
            (roll_drone, pitch_drone, yaw_drone) = euler_from_quaternion (orientation_list_drone)

            orientation_list_reference = [pos_reference_msg.pose.orientation.x, pos_reference_msg.pose.orientation.y, pos_reference_msg.pose.orientation.z, pos_reference_msg.pose.orientation.w]
            (roll_reference, pitch_reference, yaw_reference) = euler_from_quaternion (orientation_list_reference)
            
            print('error reference: ', yaw_reference)
            print('drone yaw: ', yaw_drone)
            #calculating the error
            error[0]  = pos_reference_msg.pose.position.x - trans_drone[0]
            error[1] =  pos_reference_msg.pose.position.y - trans_drone[1]
            error[2] =  pos_reference_msg.pose.position.z - trans_drone[2]

            if (yaw_reference == PI and yaw_drone < 0):
                    yaw_drone += 2*PI

            error[3] = yaw_reference - yaw_drone

            print('error: ', error[3])

            #calculating the Iterm
            Iterm[0] = constraint(Iterm[0] + error[0]*sample_time, -I_max, I_max)
            Iterm[1] = constraint(Iterm[1] + error[1]*sample_time, -I_max, I_max)
            Iterm[2] = constraint(Iterm[2] + error[2]*sample_time, -I_max, I_max)
            Iterm[3] = constraint(Iterm[3] + error[3]*sample_time, -I_max, I_max)
            print('iterm x: ', Iterm[0])
            #calculate the process variable
            v_pid[0] = (KP[0] * error[0]) + (KI[0] * Iterm[0]) + ((KD[0] * (error[0] - prev_error[0]))/sample_time)
            v_pid[1] = (KP[1] * error[1]) + (KI[1] * Iterm[1]) + ((KD[1] * (error[1] - prev_error[1]))/sample_time)
            v_pid[2] = (KP[2] * error[2]) + (KI[2] * Iterm[2]) + ((KD[2] * (error[2] - prev_error[2]))/sample_time)
            v_pid[3] = (KP[3] * error[3]) + (KI[3] * Iterm[3]) + ((KD[3] * (error[3] - prev_error[3]))/sample_time)

            #definrthe prev erroer

            prev_error[0] = error[0]
            prev_error[1] = error[1]
            prev_error[2] = error[2]
            prev_error[3] = error[3]

            if v_pid[0] > vel_max:
                v_pid[0] = vel_max
            if v_pid[1] > vel_max:
                v_pid[1] = vel_max
            if v_pid[2] > vel_max:
                v_pid[2] = vel_max
            if v_pid[3] > vel_max:
                v_pid[3] = yaw_rate_max
            
            print(v_pid[3])
            vel_cmd_msg.twist.linear.x = v_pid[0]
            vel_cmd_msg.twist.linear.y = v_pid[1]
            vel_cmd_msg.twist.linear.z = v_pid[2]
            vel_cmd_msg.twist.angular.z = v_pid[3]

            vel_pub.publish(vel_cmd_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
