#!/usr/bin/env python

import numpy as np

def main():

	dt = 10
	x0 = np.array([[2.0],[1.0]])
	P0 = 2*np.identity(2)

	# define model 
	A = np.matrix([[1, dt], [0, 1]])
	C = np.array([[1, 0]])
	Q = 0.1*np.identity(2)
	R = 0.1
	z = 2.25

	# predict
	x = A*x0 # np.multiply(A, x0)
	P = A*P0*np.transpose(A) + Q

	# update 
	S = C*P*np.transpose(C) + R
	K = P*np.transpose(C)*np.linalg.inv(S)
	x = x + K*(z - C*x)
	P = (np.identity(2) - K*C)*P
	print(x)
	print(P)


if __name__ == '__main__':
	main()
