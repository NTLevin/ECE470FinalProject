#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
 
	S1 = [0, 0, 1, 150, 150, 0] # [w, v] [R, p]
	S2 = [0, 1, 0, -162, 0, -150]
	S3 = [0, 1, 0, -162, 0, 94]
	S4 = [0, 1, 0, -162, 0, 307]
	S5 = [1, 0, 0, 0, 162, -260]
	S6 = [0, 1, 0, -162, 0, 390]
 
	# m = [0, 1, 0, -215.5, 0, -390]
	# w = m[0:3]
	# w_m = omegaTrans(w)
	# v_m = m[3:]
	# M = np.array([
    #  			[w_m[0][0], w_m[0][1], w_m[0][2], v_m[0]], 
    #            	[w_m[1][0], w_m[1][1], w_m[1][2], v_m[1]],
	# 			[w_m[2][0], w_m[2][1], w_m[2][2], v_m[2]],
    #             [0, 0, 0, 1]])
    
    # S1 = [[]]
    
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

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
 
	# print(STrans(S[0]))
	# print(STrans(S[1]))
	# print(STrans(S[2]))
	# print(STrans(S[3]))
	# print(STrans(S[4]))
	# print(STrans(S[5]))
 
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
	# print(str(T01))
	# print(str(T12))
	# print(str(T23))
	# print(str(T34))
	# print(str(T45))
	# print(str(T56))
	# STrans(M)
	T = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ M
	# T = T01.dot(T12).dot(T23).dot(T34).dot(T45).dot(T56).dot(M)

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

def omegaTrans(w):
    return np.array([[0, -w[2], w[1]], 
                     [w[2], 0, -w[0]], 
                     [-w[1], w[0], 0]])

def STrans(S):
    w = S[0:3]
    v = S[3:]
    W = omegaTrans(w)
    # print(w)
    # print(v)
    # print(W)
    BigS = np.array([[W[0][0], W[0][1], W[0][2], v[0]], 
                     [W[1][0], W[1][1], W[1][2], v[1]], 
                     [W[2][0], W[2][1], W[2][2], v[2]], 
                     [0, 0, 0, 0]])
    print(str(BigS))
    return BigS
