#!/usr/bin/env python
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
    print(S)
    # print("SHAPES:\n",q1.shape,w1.shape,S1.shape,S.shape)
    print("S1:\n", S1,S2,S3,S4,S5,S6)
    print("SSS\n",S[0],S[0][2],S[0][1],S[0][0])
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
	print("M", M)

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

'''
input theta1: 20 -45 105 -60 -90 0
output p: 0.1343 0.4275 0.009071
expect p: 0.134 0.427 0.009

input theta2: -105 -105 100 -50 -110 -30
output p: -0.1266 -0.1697 -0.3772
expect p: -0.127 -0.170 0.377
'''
