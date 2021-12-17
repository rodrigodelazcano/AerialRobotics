#!/usr/bin/env python

import time
import math
import numpy as np
import rospy
from std_msgs.msg import Int64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
import mavros_msgs.srv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

def ext_state_cb(msg):
	global ext_state_msg
	ext_state_msg = msg

def target_cb(msg):
	global target_msg
	target_msg = msg
	
def pose_cb(msg):
	global pose_msg
	pose_msg = msg

def state_cb(msg):
	global state_msg
	state_msg = msg

def detection_cb(msg):
	global first_detection_flag, stage_idx, detection_counter
	if (msg.data):
		detection_counter = detection_counter + 1

	if ((msg.data) and (first_detection_flag) and (detection_counter > 10)):
		first_detection_flag = False
		stage_idx = 2
		print("  >> Target Found, Stage 1 complete")

def vel_local_cb(msg):
	global vel_body_msg
	vel_body_msg = msg

def reached_waypoint(wp_x, wp_y, wp_z):

	global pose_msg

	dx = (pose_msg.pose.position.x - wp_x)
	dy = (pose_msg.pose.position.y - wp_y)
	dz = (pose_msg.pose.position.z - wp_z)
	distance = math.sqrt(dx*dx + dy*dy + dz*dz)
	THRESHOLD = 0.15
	flag = False

	if ((THRESHOLD > abs(dx)) and (THRESHOLD > abs(dy)) and (THRESHOLD > abs(dz))):
		flag = True

	return distance, flag

def main():

	rospy.init_node("mp_node")  
	rospy.Subscriber("/drone/kf_pos", Odometry, target_cb)
	rospy.Subscriber("/drone/tag_detected", Bool, detection_cb)
	rospy.Subscriber("/mavros/state", State, state_cb)
	rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
	rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, vel_local_cb)
	rospy.Subscriber("/mavros/extended_state", ExtendedState, ext_state_cb)

	pos_sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
	vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
	reference_pub = rospy.Publisher('/drone/reference_goal', PoseStamped, queue_size=1)
	stage_idx_pub  = rospy.Publisher("drone/stage_idx", Int64, queue_size=1)
	rate = rospy.Rate(20.0)

	global pose_msg, target_msg, target_detected, state_msg, vel_local_msg
	global stage_idx, first_detection_flag, detection_counter, ext_state_msg
	pose_msg = PoseStamped()
	target_msg = Odometry()
	state_msg  = State()
	ext_state_msg = ExtendedState()
	stage_idx_msg = Int64()
	vel_local_msg = TwistStamped()
	vel_cmd_msg = TwistStamped()
	stage_idx = int(1)
	detection_counter = 0
	target_detected = False
	first_detection_flag = True

	# search patern 
	wp = np.zeros((4,3))
	wp[0][0], wp[0][1], wp[0][2] =  0.0, 0.0, 1.0
	wp[1][0], wp[1][1], wp[1][2] =  1.0, 0.0, 1.0
	wp[2][0], wp[2][1], wp[2][2] =  0.0, 1.0, 1.0
	wp[3][0], wp[3][1], wp[3][2] =  0.0, 0.0, 1.0    
	pos_sp_msg = PoseStamped()

	wp_idx = int(0)
	number_of_waypoints, _ = wp.shape

	print("here")

	vel_cmd_msg.twist.linear.x = 0.2
	at_wp = False
	while not rospy.is_shutdown():	

		# wait for offboard mode 
		if ((state_msg.mode == "OFFBOARD") and(state_msg.armed == True)):

			if (stage_idx == 1):
				
				# vel_pub.publish(vel_cmd_msg)
				pos_sp_msg.header.stamp = rospy.Time.now()
				pos_sp_msg.pose.position.x = 0
				pos_sp_msg.pose.position.y = 0
				pos_sp_msg.pose.position.z = 1
				pos_sp_msg.pose.orientation.x = 0.0
				pos_sp_msg.pose.orientation.y = 0.0
				pos_sp_msg.pose.orientation.z = 0.0
				pos_sp_msg.pose.orientation.w = 1.0
				pos_sp_pub.publish(pos_sp_msg)
				dist, at_wp = reached_waypoint(wp[wp_idx][0], wp[wp_idx][1], wp[wp_idx][2])
				if (at_wp):
					stage_idx = 2
				print(round(dist,2), wp_idx, stage_idx)

			if stage_idx==2:

				pos_sp_msg.header.stamp = rospy.Time.now()
				pos_sp_msg.pose.position.x = 2
				pos_sp_msg.pose.position.y = 0
				pos_sp_msg.pose.position.z = 2
                                q  = quaternion_from_euler(0,0,0.7)
				pos_sp_msg.pose.orientation.x = q[0]
				pos_sp_msg.pose.orientation.y = q[1]
				pos_sp_msg.pose.orientation.z = q[2]
				pos_sp_msg.pose.orientation.w = q[3]
				reference_pub.publish(pos_sp_msg)

		else:
			pos_sp_msg.header.stamp = rospy.Time.now()
			pos_sp_msg.pose.position.x = 0
			pos_sp_msg.pose.position.y = 0
			pos_sp_msg.pose.position.z = 0
			pos_sp_pub.publish(pos_sp_msg)
		
		stage_idx_msg.data = stage_idx
		stage_idx_pub.publish(stage_idx_msg)
		rate.sleep()

	# land and disarm
	rospy.wait_for_service("/mavros/set_mode")
	flightModeService = rospy.ServiceProxy("/mavros/set_mode", mavros_msgs.srv.SetMode)
	isModeChanged = flightModeService(custom_mode="AUTO.LAND") 


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
