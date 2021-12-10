#!/usr/bin/env python

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def img_cb(msg):
	global img
	img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def main():

    rospy.init_node("aruco_node")  
    rospy.Subscriber("/tracking", Image, img_cb)

    pose_pub = rospy.Publisher("/drone/detection_raw", PoseStamped, queue_size=100)
    detection_pub = rospy.Publisher("/drone/tag_detected", Bool, queue_size=1)
    # aruco_img_pub = rospy.Publisher("/aruco/image", Image, queue_size=100)
    rate = rospy.Rate(30.0)

    dictionary = aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()


    # old drone params 
    # matrix_coefficients = np.array([[2.8335410674101769e+02, 0.0, 3.5946419216762064e+02],[0.0, 2.8400228079839877e+02, 2.2537805605509104e+02],[0.000000, 0.000000, 1.000000]])
    # distortion_coefficients = np.array([-1.6657591778798067e-02, 2.8450061335631088e-02,2.1345078848243913e-02, 5.6667823864002377e-03, 0.000000])

    # gazebo params 
    # matrix_coefficients = np.array([[554.3827128226441, 0.0, 320.5], [0.0, 554.3827128226441, 240.5], [0.0, 0.0, 1.0]])
    # distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # calibrated on Wednesday  
    matrix_coefficients = np.array([[285.875, 0.0, 308.905],[0.0, 284.853, 255.442],[0.000000, 0.000000, 1.000000]])
    distortion_coefficients = np.array([-0.024988535, 0.01199187, 0, 0, 0.000000])

    marker_length = 15.0 # cm, 22.5 on gazebo  
    detection_msg = Bool()
    detection_msg.data = False
    detection_pub.publish(detection_msg)

    global img
    img = np.zeros((480,640,3), np.uint8)

    pose_msg = PoseStamped()

    while not rospy.is_shutdown():

        # https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/aruco_pose_estimation.py
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, dictionary, parameters=parameters, cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)

        if ids is not None:
            detection_msg.data = True
            detection_pub.publish(detection_msg)
            ret = aruco.estimatePoseSingleMarkers(corners, marker_length, matrix_coefficients, distortion_coefficients)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_ct)
            #aruco.drawDetectedMarkers(img, corners)  # Draw A square around the markers
            #aruco.drawAxis(img, matrix_coefficients, distortion_coefficients, rvec, tvec, 10)  # Draw Axis
            #ros_img = CvBridge().cv2_to_imgmsg(img, "bgr8")
            #ros_img.header.frame_id = "tracking_cam"
            #ros_img.header.stamp = rospy.Time.now()
            #aruco_img_pub.publish(ros_img)        
            # must apply rotation to get local coordinates, but for now I will just publish raw reading  
            # tvec = tvec/100/2
            tvec = tvec/100

            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = tvec[0]
            pose_msg.pose.position.y = tvec[1]
            pose_msg.pose.position.z = tvec[2]
            pose_pub.publish(pose_msg)
            print(tvec)
            """
            cv2.imshow("aruco_image", img)
            if (cv2.waitKey(1) == ord("q")):
                break
            """
            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "tracking_cam"
            t.child_frame_id = "tag"
            t.transform.translation.x = tvec[0]
            t.transform.translation.y = tvec[1]
            t.transform.translation.z = tvec[2]

            q = quaternion_from_euler(roll_marker,pitch_marker,yaw_marker)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)

        rate.sleep()

	
        #cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
