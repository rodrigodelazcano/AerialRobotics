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
	stage_idx = int(1)
	detection_counter = 0
	target_detected = False
	first_detection_flag = True

	wp = np.zeros((4,3))
	wp[0][0], wp[0][1], wp[0][2] =  0.0, 0.0, 0.0
	wp[1][0], wp[1][1], wp[1][2] =  0.0, 0.0, 1.5
	wp[2][0], wp[2][1], wp[2][2] =  2, 0.0, 1.5
	wp[3][0], wp[3][1], wp[3][2] =  0.0, 0.0, 1.5    
	pos_sp_msg = PoseStamped()

	wp_idx = int(1)
	number_of_waypoints, _ = wp.shape

	delta_x = 0.5
	delta_z = 0.5
	vz = delta_z/1.0

	print("here")

	listener = tf.TransformListener()

	while not rospy.is_shutdown():	

		# wait for offboard mode 
		if ((state_msg.mode == "OFFBOARD") and(state_msg.armed == True)):

			if (stage_idx == 1):
				# proceed to the next waypoint
				dist, at_wp = reached_waypoint(wp[wp_idx][0], wp[wp_idx][1], wp[wp_idx][2])
				if (at_wp):
					wp_idx = wp_idx + 1
					if wp_idx > number_of_waypoints-1:
						break
				else:
					print(round(dist,2), wp_idx, stage_idx)
					pos_sp_msg.header.stamp = rospy.Time.now()
					pos_sp_msg.pose.position.x = wp[wp_idx][0]
					pos_sp_msg.pose.position.y = wp[wp_idx][1]
					pos_sp_msg.pose.position.z = wp[wp_idx][2]
					pos_sp_pub.publish(pos_sp_msg)
			
			# maneuver behind target
			if (stage_idx == 2):

				try:
					(trans, rot) = listener.lookupTransform('/world', '/goal_behind_tag', rospy.Time(0))
					pos_sp_msg.pose.position.x = trans[0]
					pos_sp_msg.pose.position.y = trans[1]
					pos_sp_msg.pose.position.z = trans[2]

					pos_sp_msg.pose.orientation.x = rot[0]
					pos_sp_msg.pose.orientation.y = rot[1]
					pos_sp_msg.pose.orientation.z = rot[2]
					pos_sp_msg.pose.orientation.w = rot[3]
			
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):	
					pos_sp_msg.pose.position.x = target_msg.pose.pose.position.x - delta_x
					pos_sp_msg.pose.position.y = target_msg.pose.pose.position.y
					pos_sp_msg.pose.position.z = target_msg.pose.pose.position.z + delta_z
					pos_sp_msg.pose.orientation.x = target_msg.pose.pose.orientation.x
					pos_sp_msg.pose.orientation.y = target_msg.pose.pose.orientation.y
					pos_sp_msg.pose.orientation.z = target_msg.pose.pose.orientation.z
					pos_sp_msg.pose.orientation.w = target_msg.pose.pose.orientation.w
				dist, at_wp = reached_waypoint(pos_sp_msg.pose.position.x, pos_sp_msg.pose.position.y, pos_sp_msg.pose.position.z)
				if (at_wp):
					stage_idx = stage_idx + 1
					print("  >> Stage 2 complete")
				else:
					pos_sp_msg.header.stamp = rospy.Time.now()
					pos_sp_pub.publish(pos_sp_msg)	
					print(round(dist,2), stage_idx)		

			# maneuver above target 
			if (stage_idx == 3):
				try:
					(trans, rot) = listener.lookupTransform('/world', '/goal_over_tag', rospy.Time(0))
					pos_sp_msg.pose.position.x = trans[0] 
					pos_sp_msg.pose.position.y = trans[1]
					pos_sp_msg.pose.position.z = trans[2] + 0.5

					pos_sp_msg.pose.orientation.x = rot[0]
					pos_sp_msg.pose.orientation.y = rot[1]
					pos_sp_msg.pose.orientation.z = rot[2]
					pos_sp_msg.pose.orientation.w = rot[3]
			
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):	
					pos_sp_msg.pose.position.x = target_msg.pose.pose.position.x
					pos_sp_msg.pose.position.y = target_msg.pose.pose.position.y
					pos_sp_msg.pose.position.z = target_msg.pose.pose.position.z + delta_z
					pos_sp_msg.pose.orientation.x = target_msg.pose.pose.orientation.x
					pos_sp_msg.pose.orientation.y = target_msg.pose.pose.orientation.y
					pos_sp_msg.pose.orientation.z = target_msg.pose.pose.orientation.z
					pos_sp_msg.pose.orientation.w = target_msg.pose.pose.orientation.w

				dist, at_wp = reached_waypoint(pos_sp_msg.pose.position.x, pos_sp_msg.pose.position.y, pos_sp_msg.pose.position.z)
				if (at_wp):
					stage_idx = stage_idx + 1
					print("  >> Stage 3 complete")
					t4 = time.time()
				else:
					pos_sp_msg.header.stamp = rospy.Time.now()
					pos_sp_pub.publish(pos_sp_msg)	
					print(round(dist,2), stage_idx)	

			# slowly descend onto target 
			if (stage_idx == 4):
				print("  >> Beginning final descent...")
				time_to_descend = 5.0
				t4 = time.time()
				landed = False

				# continue descending until on the ground 
				while not landed:

					try:
						(trans, rot) = listener.lookupTransform('/world', '/goal_over_tag', rospy.Time(0))
						pos_sp_msg.pose.position.x = trans[0] 
						pos_sp_msg.pose.position.y = trans[1]
						pos_sp_msg.pose.position.z = trans[2] + 0.5 - vz*(time.time() - t4)

						pos_sp_msg.pose.orientation.x = rot[0]
						pos_sp_msg.pose.orientation.y = rot[1]
						pos_sp_msg.pose.orientation.z = rot[2]
						pos_sp_msg.pose.orientation.w = rot[3]
			
					except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):	
						pos_sp_msg.pose.position.x = target_msg.pose.pose.position.x
						pos_sp_msg.pose.position.y = target_msg.pose.pose.position.y
						pos_sp_msg.pose.position.z = target_msg.pose.pose.position.z + delta_z - vz*(time.time() - t4)
						pos_sp_msg.pose.orientation.x = target_msg.pose.pose.orientation.x
						pos_sp_msg.pose.orientation.y = target_msg.pose.pose.orientation.y
						pos_sp_msg.pose.orientation.z = target_msg.pose.pose.orientation.z
						pos_sp_msg.pose.orientation.w = target_msg.pose.pose.orientation.w
					
					
					pos_sp_msg.header.stamp = rospy.Time.now()
					pos_sp_pub.publish(pos_sp_msg)	
					
					# either descend for some fixed time or detect when on ground 
					if ((time.time() - t4 > time_to_descend) or (ext_state_msg.landed_state == 1)):
						landed = True
						stage_idx = stage_idx + 1

					rate.sleep()

			if (stage_idx == 5):
				print("  >> Stage 4 complete")
				break

		else:
			pos_sp_msg.header.stamp = rospy.Time.now()
			pos_sp_msg.pose.position.x = 0.0
			pos_sp_msg.pose.position.y = 0.0
			pos_sp_msg.pose.position.z = 0.0
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
