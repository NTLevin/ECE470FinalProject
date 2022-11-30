#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	# M = np.eye(4)
	# S = np.zeros((6,6))

	

	M = np.array([[0, -1, 0, 0.39], [0, 0, -1,0.401], [1, 0, 0, 0.2155], [0, 0, 0, 1]])

	q1 = np.transpose([-.150, .150, 0])
	q2 = np.transpose([q1[0], 0, .010 + .152])
	q3 = np.transpose([q2[0] + .244, 0, q2[2]])
	q4 = np.transpose([q3[0] + .213, 0, q3[2]])
	q5 = np.transpose([0, 0.150 + .120 -0.093 + .083, q4[2]])
	q6 = np.transpose([-.150 + .540, 0, q5[2]]) 

	w1 = np.transpose([0, 0, 1])
	w2 = np.transpose([0, 1, 0])
	w3 = np.transpose([0, 1, 0])
	w4 = np.transpose([0, 1, 0])
	w5 = np.transpose([1, 0, 0])
	w6 = np.transpose([0, 1, 0])
	
	v1 = np.cross(q1,w1)
	v2 = np.cross(q2,w2)
	v3 = np.cross(q3,w3)
	v4 = np.cross(q4,w4)
	v5 = np.cross(q5,w5)
	v6 = np.cross(q6,w6)

	S1 = np.array([[0,-w1[2],w1[1],v1[0]],
					[w1[2],0,-w1[0],v1[1]]
					,[-w1[1], w1[0], 0,v1[2]],
					[0, 0, 0, 0]])


	S2 = np.array([[0,-w2[2],w2[1], v2[0]],
					[w2[2], 0, -w2[0], v2[1]]
					,[-w2[1], w2[0], 0, v2[2]],
					[0, 0, 0, 0]])


	S3 = np.array([[0,-w3[2],w3[1], v3[0]],
					[w3[2], 0, -w3[0], v3[1]]
					,[-w3[1], w3[0], 0, v3[2]],
					[0, 0, 0, 0]])


	S4 = np.array([[0,-w4[2],w4[1], v4[0]],
					[w4[2], 0, -w4[0], v4[1]]
					,[-w4[1], w4[0], 0, v4[2]],
					[0, 0, 0, 0]])



	S5 = np.array([[0,-w5[2],w5[1], v5[0]],
					[w5[2], 0, -w5[0], v5[1]]
					,[-w5[1], w5[0], 0, v5[2]],
					[0, 0, 0, 0]])


	S6 = np.array([[0,-w6[2],w6[1], v6[0]],
					[w6[2], 0, -w6[0], v6[1]]
					,[-w6[1], w6[0], 0, v6[2]],
					[0, 0, 0, 0]])



	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	# T = np.eye(4)

	# M, S = Get_MS()
	q1 = np.transpose([-.150, .150, 0])
	q2 = np.transpose([q1[0], 0, .010 + .152])
	q3 = np.transpose([q2[0] + .244, 0, q2[2]])
	q4 = np.transpose([q3[0] + .213, 0, q3[2]])
	q5 = np.transpose([0, 0.150 + .120 -0.093 + .083, q4[2]])
	q6 = np.transpose([-.150 + .540, 0, q5[2]]) 

	w1 = np.transpose([0, 0, 1])
	w2 = np.transpose([0, 1, 0])
	w3 = np.transpose([0, 1, 0])
	w4 = np.transpose([0, 1, 0])
	w5 = np.transpose([1, 0, 0])
	w6 = np.transpose([0, 1, 0])
	
	v1 = np.cross(q1,w1)
	v2 = np.cross(q2,w2)
	v3 = np.cross(q3,w3)
	v4 = np.cross(q4,w4)
	v5 = np.cross(q5,w5)
	v6 = np.cross(q6,w6)

	S1 = np.array([[0,-w1[2],w1[1],v1[0]],
					[w1[2],0,-w1[0],v1[1]]
					,[-w1[1], w1[0], 0,v1[2]],
					[0, 0, 0, 0]])


	S2 = np.array([[0,-w2[2],w2[1], v2[0]],
					[w2[2], 0, -w2[0], v2[1]]
					,[-w2[1], w2[0], 0, v2[2]],
					[0, 0, 0, 0]])


	S3 = np.array([[0,-w3[2],w3[1], v3[0]],
					[w3[2], 0, -w3[0], v3[1]]
					,[-w3[1], w3[0], 0, v3[2]],
					[0, 0, 0, 0]])


	S4 = np.array([[0,-w4[2],w4[1], v4[0]],
					[w4[2], 0, -w4[0], v4[1]]
					,[-w4[1], w4[0], 0, v4[2]],
					[0, 0, 0, 0]])



	S5 = np.array([[0,-w5[2],w5[1], v5[0]],
					[w5[2], 0, -w5[0], v5[1]]
					,[-w5[1], w5[0], 0, v5[2]],
					[0, 0, 0, 0]])


	S6 = np.array([[0,-w6[2],w6[1], v6[0]],
					[w6[2], 0, -w6[0], v6[1]]
					,[-w6[1], w6[0], 0, v6[2]],
					[0, 0, 0, 0]])
	# print("S1:", S1)
	# print("S2:", S2)
	# print("S3:", S3)
	# print("S4:", S4)
	# print("S5:", S5)
	# print("S6:", S6)

	M = np.array([[0, -1, 0, 0.39], [0, 0, -1,0.401], [1, 0, 0, 0.2155], [0, 0, 0, 1]])
	print(M)



	# T = np.matmul(expm(np.multiply(S1,theta1)), np.matmul(expm(np.multiply(S2,theta2)), np.matmul(expm(np.multiply(S3,theta3)), np.matmul(expm(np.multiply(S4,theta4)), np.matmul(expm(np.multiply(S5,theta5)), np.matmul(expm(np.multiply(S6,theta6)), M))))))
	T = expm(S1 * theta1) @ expm(S2 * theta2) @ expm(S3 * theta3) @ expm(S4 * theta4) @ expm(S5 * theta5) @ expm(S6 * theta6) @ M  

	print("T :", T)


	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0

	L1 = 0.152 
	L2 = 0.120
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059


	xBgrip = xWgrip + 0.150 
	yBgrip = yWgrip  - 0.150
	zBGrip = zWgrip - 0.010
	yaw = np.deg2rad(yaw_WgripDegree)


	xC = xBgrip - L9 * np.cos(yaw)
	yC = yBgrip - L9 * np.sin(yaw)
	zC = zBGrip
	print((L2 + L6 - L4)/(np.sqrt(xC**2 + yC**2)))
 
	


	theta1 = np.arctan2(yC,xC) - np.arcsin((L2 + L6 - L4)/(np.sqrt(xC**2 + yC**2)))

	theta6 = np.deg2rad(90) - yaw + theta1

	d_67 = np.sqrt((L7)**2 + ((L6+0.027))**2)
	Beta = np.arctan2((L7),(L6 + 0.027))
	
	x3_end = xC + d_67 * np.sin(theta1 - Beta)
	y3_end = yC - d_67 * np.cos(theta1 - Beta)
	z3_end = zC + L10 + L8

	
	
	theta5 = np.deg2rad(-90)


	d35 = np.sqrt((z3_end - L1)**2 + (x3_end**2+y3_end**2))
	theta3 = np.deg2rad(180) - np.arccos((L3**2 + L5**2 - d35**2)/(2*L3*L5))

	d5 = L5*np.cos(theta3)
	d6 = L5*np.sin(theta3)
	# theta2 = -(np.arctan2(d6,(L3+d5)) + np.arctan2((z3_end - L1),np.sqrt(x3_end**2 + y3_end**2)))
	theta2 = -(np.arcsin(L5 * np.sin(theta3) / d35) + np.arctan2((z3_end - L1),np.sqrt(x3_end**2 + y3_end**2)))
 
	theta4 = -theta3 - theta2


	print("theta 1: ", theta1)
	print("theta 2: ", theta2)
	print("theta 3: ", theta3)
	print("theta 4: ", theta4)
	print("theta 5: ", theta5)
	print("theta 6: ", theta6)
 
	# theta1 = np.deg2rad(theta1)
	# theta2 = np.deg2rad(theta2)
	# theta3 = np.deg2rad(theta3)
	# theta4 = np.deg2rad(theta4)
	# theta5 = np.deg2rad(theta5)
	# theta6 = np.deg2rad(theta6)
 	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
