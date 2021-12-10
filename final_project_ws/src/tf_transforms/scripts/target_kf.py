#!/usr/bin/env python

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from std_msgs.msg import Float64, Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped,TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros

import roslib
import tf

def half_kf(x0, P0):
	# define model 
	global dt
	A = np.matrix([[1, dt], [0, 1]])
	C = np.array([[1, 0]])
	Q = 0.1*np.identity(2)
	R = 0.1

	# predict
	x = A*x0 # np.multiply(A, x0)
	P = A*P0*np.transpose(A) + Q

	return x, P 

def full_kf(x0, z, P0):

	# define model 
	global dt
	A = np.matrix([[1, dt], [0, 1]])
	C = np.array([[1, 0]])
	Q = 0.1*np.identity(2)
	R = 0.1

	# predict
	x = A*x0 # np.multiply(A, x0)
	P = A*P0*np.transpose(A) + Q

	# update 
	S = C*P*np.transpose(C) + R
	K = P*np.transpose(C)*np.linalg.inv(S)
	x = x + K*(z - C*x)
	P = (np.identity(2) - K*C)*P

	return x, P

def stage_idx_cb(msg):
	global stage_idx
	stage_idx = msg.data

def main():

	rospy.init_node("target_kf_node")  
	rospy.Subscriber("/drone/stage_idx", Int64, stage_idx_cb)
	
	kf_pub = rospy.Publisher("/drone/kf_pos", Odometry, queue_size=1)
	target_raw_pub = rospy.Publisher("drone/target_raw", PointStamped, queue_size=1)
	target_yaw_raw_pub = rospy.Publisher("/dorne/target_yaw_raw", Float64, queue_size=1)

	tf_br  = tf.TransformBroadcaster()

	rate = rospy.Rate(20.0)
	listener = tf.TransformListener()

	global target_raw_msg, dt, stage_idx
	target_raw_msg = PointStamped()
	dt = (1/20.0)

	xhat = np.array([[0.0],[0.0]])
	yhat = np.array([[0.0],[0.0]])
	zhat = np.array([[0.0],[0.0]])

	yawhat = np.array([[0.0],[0.0]])

	Px = np.identity(2)
	Py = np.identity(2)
	Pz = np.identity(2)

	Pyaw = np.identity(2)

	kf_msg = Odometry()

	while not rospy.is_shutdown():

		try:
			(trans, rot) = listener.lookupTransform('/world', '/tag', rospy.Time(0))		
			xm = trans[0]
			ym = trans[1]
			zm = trans[2]
			orientation_list = [rot[0], rot[1], rot[2], rot[3]]
			(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
			yawm = yaw

			target_raw_msg.header.stamp = rospy.Time.now()
			target_raw_msg.point.x = xm
			target_raw_msg.point.y = ym
			target_raw_msg.point.z = zm
			target_raw_pub.publish(target_raw_msg)
			target_yaw_raw_pub.publish(yawm)
			xhat, Px = full_kf(xhat, xm, Px)
			yhat, Py = full_kf(yhat, ym, Py)
			zhat, Pz = full_kf(zhat, zm, Pz)

			yawhat, Pyaw = full_kf(yawhat, yawm, Pyaw)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			xhat, Px = half_kf(xhat, Px)
			yhat, Py = half_kf(yhat, Py)
			zhat, Pz = half_kf(zhat, Pz)

			yawhat, Pyaw = half_kf(yawhat, Pyaw)
			

		kf_msg.header.stamp = rospy.Time.now()
		kf_msg.header.frame_id = "kf_tag"
		kf_msg.pose.pose.position.x = xhat[0,0]
		kf_msg.pose.pose.position.y = yhat[0,0]
		kf_msg.pose.pose.position.z = zhat[0,0]

		q = quaternion_from_euler(0, 0, yawhat[0,0]+1.57079632679)
		kf_msg.pose.pose.orientation.x = q[0]
		kf_msg.pose.pose.orientation.y = q[1]
		kf_msg.pose.pose.orientation.z = q[2]
		kf_msg.pose.pose.orientation.w = q[3]

		kf_msg.twist.twist.linear.x = xhat[1,0]
		kf_msg.twist.twist.linear.y = yhat[1,0]
		kf_msg.twist.twist.linear.z = zhat[1,0]

		kf_msg.twist.twist.angular.z = yawhat[1,0]

		print(round(xhat[0,0],2), round(yhat[0,0],2), round(zhat[0,0],2))
		kf_pub.publish(kf_msg)

		br = tf2_ros.TransformBroadcaster()
		t_over_tag = TransformStamped()

		t_over_tag.header.stamp = rospy.Time.now()
		t_over_tag.header.frame_id = "world"
		t_over_tag.child_frame_id = "goal_over_tag"
		t_over_tag.transform.translation.x = kf_msg.pose.pose.position.x
		t_over_tag.transform.translation.y = kf_msg.pose.pose.position.y
		t_over_tag.transform.translation.z = kf_msg.pose.pose.position.z

		t_over_tag.transform.rotation.x = q[0]
		t_over_tag.transform.rotation.y = q[1]
		t_over_tag.transform.rotation.z = q[2]
		t_over_tag.transform.rotation.w = q[3]

		br.sendTransform(t_over_tag)

		t_behind_tag = TransformStamped()
		t_behind_tag.header.stamp = rospy.Time.now()
		t_behind_tag.header.frame_id = "goal_over_tag"
		t_behind_tag.child_frame_id = "goal_behind_tag"
		t_behind_tag.transform.translation.x = -0.5
		t_behind_tag.transform.translation.y = 0.0
		t_behind_tag.transform.translation.z = 0.5

		t_behind_tag.transform.rotation.x = 0
		t_behind_tag.transform.rotation.y = 0
		t_behind_tag.transform.rotation.z = 0
		t_behind_tag.transform.rotation.w = 1

		br.sendTransform(t_behind_tag)
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
