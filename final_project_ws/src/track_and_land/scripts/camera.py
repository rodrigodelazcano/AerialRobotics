#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():

	rospy.init_node("camera_node")  
	rate = rospy.Rate(20.0)

	img_pub = rospy.Publisher("/camera/img_raw", Image, queue_size=1)

	cam = cv2.VideoCapture(0)
	cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
	bridge = CvBridge()

	while not rospy.is_shutdown():
		cap, img = cam.read()

		if cap:
			img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
			img_pub.publish(img_msg)
			
			"""
			cv2.imshow("img", img)
			if (cv2.waitKey(1) == ord("q")):
				break
			"""
		rate.sleep()

	cam.release()
	cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
