#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
    w1=np.array([0,0,1])
    q1=np.array([-.150,.150,.0])
    w2=np.array([0,1,0])
    q2=np.array([-.150,.270,.162])
    w3=np.array([0,1,0])
    q3=np.array([.094,.270,.162])
    w4=np.array([0,1,0])
    q4=np.array([.307,.177,.162])
    w5=np.array([1,0,0])
    q5=np.array([.307,.260,.162])
    w6=np.array([0,1,0])
    q6=np.array([.39,.260,.162])
    V1=np.cross(q1,w1)
    V2=np.cross(q2,w2)
    V3=np.cross(q3,w3)
    V4=np.cross(q4,w4)
    V5=np.cross(q5,w5)
    V6=np.cross(q6,w6)
    S1=np.concatenate((w1,V1),axis=None)
    S2=np.concatenate((w2,V2),axis=None)
    S3=np.concatenate((w3,V3),axis=None)
    S4=np.concatenate((w4,V4),axis=None)
    S5=np.concatenate((w5,V5),axis=None)
    S6=np.concatenate((w6,V6),axis=None)
    M= np.array([[0,-1, 0,.39], [0,0,-1,.401],[1,0,0,.2155],[0,0,0,1]])
    S=np.array([S1,S2,S3,S4,S5,S6])
    # print(S)
    # print("SHAPES:\n",q1.shape,w1.shape,S1.shape,S.shape)
    # print("S1:\n", S1,S2,S3,S4,S5,S6)
    # print("SSS\n",S[0],S[0][2],S[0][1],S[0][0])
	# ==============================================================#
    return M, S

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	# print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
	tmps = []
	for i in range(6):
		tmp = np.zeros((4, 4))
		tmp[0][1] = -S[i][2] #w3
		tmp[1][0] = S[i][2]
		tmp[0][2] = S[i][1] #w2
		tmp[2][0] = -S[i][1]
		tmp[1][2] = -S[i][0] #w1
		tmp[2][1] = S[i][0]
		tmp[0][3] = S[i][3] #v1
		tmp[1][3] = S[i][4] #v2
		tmp[2][3] = S[i][5] #v3
		tmps.append(tmp)
	theta = [theta1, theta2, theta3, theta4, theta5, theta6]
	result = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
	for i in range(6):
		# print("check:", tmps[i], theta[i])
		tmps[i] = tmps[i] * theta[i]
		# print("after:", tmps[i])
		result = np.matmul(result, expm(tmps[i]))
		# print("RESULTS:", result)
	# result = expm(tmps[0]) @ expm(tmps[1]) @ expm(tmps[2]) @ expm(tmps[3]) @ expm(tmps[4]) @ expm(tmps[5]) @ M
	result = np.matmul(result, M)
	T = result
	# print("M", M)

	# ==============================================================#

	# print(str(T) + "\n")

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
	l = 0.0535
	xWcen = xWgrip - l * np.sin(yaw_WgripDegree / 180 * np.pi)
	yWcen = yWgrip - l * np.cos(yaw_WgripDegree / 180 * np.pi)	
	zWcen = zWgrip
	l1 = (xWcen ** 2 + yWcen ** 2) ** 0.5
	tmptheta1 = np.arcsin(0.11/l1)/np.pi * 180
	tmptheta2 = np.arctan(yWcen/xWcen)/np.pi * 180
	tmpl = (l1 ** 2 - 0.11 ** 2) ** 0.5
	l2 = tmpl - 0.083
	print(l2)
 
	theta1 = (tmptheta2 - tmptheta1) / 180 * np.pi
 
 
 
	x3end = np.cos(theta1) * l2
	y3end = np.sin(theta1) * l2
	z3end = zWcen + 0.165
 
 

 
	tmptheta3 = np.arccos((0.244 ** 2 + 0.213 ** 2 - x3end ** 2 - (z3end - 0.152) ** 2) / (2 * 0.244 * 0.213)) 

	theta3 = (180 - tmptheta3/np.pi * 180)/180 * np.pi

	tmptheta4 = np.arccos((0.244 ** 2 + x3end ** 2 + (z3end - 0.152) ** 2 - 0.213 ** 2) / (2 * 0.244 * (x3end ** 2 + (z3end - 0.152) ** 2) ** 0.5)) / np.pi * 180
	tmptheta5 = np.arctan(x3end / (z3end - 0.152)) / np.pi * 180

 
	
 
	theta4 = -(tmptheta5 - (90 - tmptheta4))/180 * np.pi
 
	theta2 = -(360 - 90 - 90 + theta4 - tmptheta3/np.pi * 180)/180 * np.pi
 
	theta5 = -np.pi/2
	theta6 = (90 - yaw_WgripDegree + theta1)/180 * np.pi
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
