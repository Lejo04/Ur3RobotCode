#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([147.81, -104.79, 103.27, -89.94, -90.38, 50.16])

# Hanoi tower location 1
Q11 = [129.73*pi/180.0, -81.89*pi/180.0, 136.64*pi/180.0, -144.96*pi/180.0, -90*pi/180.0, 34.03*pi/180.0]
Q12 = [129.76*pi/180.0, -69.64*pi/180.0, 139.74*pi/180.0, -160.30*pi/180.0, -89.99*pi/180.0, 33.99*pi/180.0]
Q13 = [129.70*pi/180.0, -58.87*pi/180.0, 142.19*pi/180.0, -176.96*pi/180.0, -89.89*pi/180.0, 33.*pi/180.0]
# Hanoi tower location 2
Q21 = [149.17*pi/180.0, -84.77*pi/180.0, 138.41*pi/180.0, -137.91*pi/180.0, -91.90*pi/180.0, 50.52*pi/180.0]
Q22 = [149.20*pi/180.0, -76.68*pi/180.0, 140.99*pi/180.0, -148.57*pi/180.0, -91.89*pi/180.0, 50.50*pi/180.0]
Q23 = [148.93*pi/180.0, -64.82*pi/180.0, 144.62*pi/180.0, -164.09*pi/180.0, -91.91*pi/180.0, 50.16*pi/180.0]
Q20=  [146.10*pi/180.0, -104.36*pi/180.0, 118.38*pi/180.0, -101.40*pi/180.0, -91.80*pi/180.0, 26.49*pi/180.0]
# Hanoi tower location 3
Q31 = [169.34*pi/180.0, -82.80*pi/180.0, 138.89*pi/180.0, -146.34*pi/180.0, -90*pi/180.0, 49.44*pi/180.0]
Q32 = [169.37*pi/180.0, -71.66*pi/180.0, 141.71*pi/180.0, -160.31*pi/180.0, -90.05*pi/180.0, 49.41*pi/180.0]
Q33 = [169.55*pi/180.0, -61.34*pi/180.0, 143.04*pi/180.0, -171.97*pi/180.0, -90.04*pi/180.0, 49.54*pi/180.0]
Q30=  [169.39*10*pi/180.0, -101.82*pi/180.0, 115.96*pi/180.0, -101.12*pi/180.0, -90.65*pi/180.0, 49.75*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33]
      ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_state(msg):
    global digital_in_0
    global analog_in_0
    if(msg.DIGIN):
        digital_in_0=1
    else:
        digital_in_0=0
    
############### Your Code End Here ###############
"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    time.sleep(1)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        time.sleep(1)

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".
    error = 0
    move_arm(pub_cmd, loop_rate, Q[start_loc][2 - start_height], 4.0, 4.0)
    error = gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if error:
        return error  
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][2 - end_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    return error
############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    my_gripper_state=rospy.Subscriber('ur3/gripper',gripper_input,gripper_state)
    ############### Your Code End Here ###############
    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    input_start = 0
    input_done=0
    while(not input_done):
        input_string = input("Enter number for start <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            input_start = 1
            input_done=1
        elif (int(input_string) == 2):
            input_start = 2
            input_done=1
        elif (int(input_string) == 3):
            input_start = 3
            input_done=1
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
    ############### Your Code End Here ###############
    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")
    rospy.loginfo("Sending Goals ...")
    loop_rate = rospy.Rate(SPIN_RATE)
    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input
    loop_count=1
    while(loop_count > 0):  
        move_arm(pub_command, input_start, home, 4.0, 4.0)
        if(input_start==1):
            rospy.loginfo("Sending goal 1 ...")
            move_block(pub_command,loop_rate, input_start-1,input_start+1,input_start+1,input_start-1)
            move_block(pub_command,loop_rate,input_start-1,input_start,input_start,input_start-1)
            move_block(pub_command,loop_rate,input_start+1,input_start-1,input_start,input_start)
            move_block(pub_command,loop_rate,input_start-1,input_start-1,input_start+1,input_start-1)
            move_block(pub_command,loop_rate,input_start,input_start,input_start-1,input_start-1)
            move_block(pub_command,loop_rate,input_start,input_start-1,input_start+1,input_start)
            move_block(pub_command,loop_rate,input_start-1,input_start-1,input_start+1,input_start+1)
        if(input_start==2):
            rospy.loginfo("Sending goal 2 ...")
            move_block(pub_command,loop_rate, input_start-1,input_start,input_start,input_start-2)
            move_block(pub_command,loop_rate, input_start-1,input_start-1,input_start-2,input_start-2)
            move_block(pub_command,loop_rate, input_start,input_start-2,input_start-2,input_start-1)
            move_block(pub_command,loop_rate, input_start-1,input_start-2,input_start,input_start-2)
            move_block(pub_command,loop_rate, input_start-2,input_start-1,input_start-1,input_start-2)
            move_block(pub_command,loop_rate, input_start-2,input_start-2,input_start,input_start-1)
            move_block(pub_command,loop_rate, input_start-1,input_start-2,input_start,input_start)
        if(input_start==3):
            rospy.loginfo("Sending goal 2 ...")
            move_block(pub_command,loop_rate, input_start-1,input_start-1,input_start-3,input_start-3)
            move_block(pub_command,loop_rate, input_start-1,input_start-2,input_start-2,input_start-3)
            move_block(pub_command,loop_rate, input_start-3,input_start-3,input_start-2,input_start-1)
            move_block(pub_command,loop_rate, input_start-1,input_start-3,input_start-3,input_start-3)
            move_block(pub_command,loop_rate, input_start-2,input_start-2,input_start-1,input_start-3)
            move_block(pub_command,loop_rate, input_start-2,input_start-3,input_start-3,input_start-2)
            move_block(pub_command,loop_rate, input_start-1,input_start-3,input_start-3,input_start-1)
        
        loop_count = loop_count - 1
    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
