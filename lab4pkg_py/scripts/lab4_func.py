#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	# M = np.eye(4)
	# S = np.zeros((6,6))
	S1 = [0, 0, 1, 150, 150, 0] # [w, v] [R, p]
	S2 = [0, 1, 0, -162, 0, -150]
	S3 = [0, 1, 0, -162, 0, 94]
	S4 = [0, 1, 0, -162, 0, 307]
	S5 = [1, 0, 0, 0, 162, -260]
	S6 = [0, 1, 0, -162, 0, 390]

	S = [S1, S2, S3, S4, S5, S6]
 
	M = np.array([[0, -1, 0, 390],
      	[0, 0, -1, 401],
       	[1, 0, 0, 215.5],
        [0, 0, 0, 1]])

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

	M, S = Get_MS()
	T01 = expm(theta1 * STrans(S[0]))
	print("T01 = \n" + str(T01) + "\n")
	T12 = expm(theta2 * STrans(S[1]))
	print("T12 = \n" + str(T12) + "\n")
	T23 = expm(theta3 * STrans(S[2]))
	print("T23 = \n" + str(T23) + "\n")
	T34 = expm(theta4 * STrans(S[3]))
	print("T34 = \n" + str(T34) + "\n")
	T45 = expm(theta5 * STrans(S[4]))
	print("T45 = \n" + str(T45) + "\n")
	T56 = expm(theta6 * STrans(S[5]))
	print("T56 = \n" + str(T56) + "\n")
	print("M = \n" + str(M) + "\n")

	T = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ M

	print(str(T) + "\n")
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
	theta5 = -90.0
	theta6 = 0.0
 
	yaw_WgripDegree = np.radians(yaw_WgripDegree)
 
	xGrip = xWgrip + 0.15
	yGrip = yWgrip - 0.15
	zGrip = zWgrip - 0.01

	xCenter = xGrip - 0.0535 * np.cos(yaw_WgripDegree)
	yCenter = yGrip - 0.0535 * np.sin(yaw_WgripDegree)
	zCenter = zGrip
 
	print(f'xCenter = {xCenter}')
	print(f'yCenter = {yCenter}')
	print(f'zCenter = {zCenter}')

	d_xy = (xCenter**2 + yCenter**2)**0.5
	print(f'd_xy = {d_xy}')
	print(f'math.asin(110 / d_xy) = {math.asin(0.11 / d_xy)}')
	theta_Cen = math.asin(0.11 / d_xy)
	# print(f'math.asin(110 / d_xy) = {math.asin(110 / d_xy)}')
	theta1 = math.asin(yCenter / d_xy) - theta_Cen
 
	theta6 = (90.0 * math.pi / 180.0 - yaw_WgripDegree) + theta1
	
	x_3end = math.cos(theta1) * (d_xy * math.cos(theta_Cen) - 0.083)
	y_3end = math.sin(theta1) * (d_xy * math.cos(theta_Cen) - 0.083)
	z_3end = zCenter + 0.141
	
	d_xz = (x_3end**2 + y_3end**2 + (z_3end-0.152)**2)**0.5
	print(f'd_xz = {d_xz}')
 
	theta3_supplement = math.acos( ((-1)*d_xz**2 + 0.244**2 + 0.213**2) / (2*0.244*0.213) )
	theta3 = math.pi - theta3_supplement
	print(f'theta3_supplement = {theta3_supplement}')
 
	theta_big = math.acos( ((-1)*0.213**2 + 0.244**2 + d_xz**2) / (2*0.244*d_xz) )
 
	# theta_small = math.acos(x_3end / d_xz)
	theta_small = math.asin((z_3end-0.152) / d_xz)
	theta2 = (theta_big + theta_small) * (-1.0)
	theta4 = (theta3 - (theta_big + theta_small)) * (-1.0)
	
	theta5 *= (math.pi / 180.0)
 
	print(f'theta1 = {theta1}')
	print(f'theta2 = {theta2}')
	print(f'theta3 = {theta3}')
	print(f'theta4 = {theta4}')
	print(f'theta5 = {theta5}')
	print(f'theta6 = {theta6}')
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

def omegaTrans(w):
    return np.array([[0, -w[2], w[1]], 
                     [w[2], 0, -w[0]], 
                     [-w[1], w[0], 0]])

def STrans(S):
    w = S[0:3]
    v = S[3:]
    W = omegaTrans(w)
    BigS = np.array([[W[0][0], W[0][1], W[0][2], v[0]], 
                     [W[1][0], W[1][1], W[1][2], v[1]], 
                     [W[2][0], W[2][1], W[2][2], v[2]], 
                     [0, 0, 0, 0]])
    print(str(BigS))
    return BigS