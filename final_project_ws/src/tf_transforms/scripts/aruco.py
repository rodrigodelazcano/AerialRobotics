#!/usr/bin/env python

import cv2
import cv2.aruco as aruco
import numpy as np

dictionary = aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
parameters = aruco.DetectorParameters_create()

matrix_coefficients = np.array([[279.784498, 0.000000, 327.640456],[0.000000, 277.232322, 209.988450],[0.000000, 0.000000, 1.000000]])
distortion_coefficients = np.array([-0.258962, 0.050357, -0.000475, 0.001398, 0.000000])
marker_length = 150 # cm

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)


while True:
	cap, img = cam.read()

	if cap:
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		corners, ids, rejected_img_points = aruco.detectMarkers(gray, dictionary, parameters=parameters, cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)

		if ids is not None:
			ret = aruco.estimatePoseSingleMarkers(corners, marker_length, matrix_coefficients, distortion_coefficients)

			#-- Unpack the output, get only the first
			rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

			print(tvec)
			aruco.drawDetectedMarkers(img, corners)  # Draw A square around the markers
			aruco.drawAxis(img, matrix_coefficients, distortion_coefficients, rvec, tvec, 10)  # Draw Axis

			cv2.imshow("img", img)


	if (cv2.waitKey(1) == ord("q")):
		break

cam.release()
cv2.destroyAllWindows()