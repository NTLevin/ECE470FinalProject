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
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

Q00 = [156.45*pi/180.0, -45.91*pi/180.0, 96.95*pi/180.0, -139.61*pi/180.0, -89.81*pi/180.0, 67.63*pi/180.0]
Q01 = [156.36*pi/180.0, -52*pi/180.0, 96.20*pi/180.0, -132.75*pi/180.0, -89.80*pi/180.0, 67.59*pi/180.0]
Q02 = [156.35*pi/180.0, -56.91*pi/180.0, 94.07*pi/180.0, -125.72*pi/180.0, -89.81*pi/180.0, 67.63*pi/180.0]
Q10 = [163.17*pi/180.0, -46.47*pi/180.0, 98.19*pi/180.0, -140.36*pi/180.0, -89.64*pi/180.0, 74.34*pi/180.0]
Q11 = [162.98*pi/180.0, -52.33*pi/180.0, 97.41*pi/180.0, -133.73*pi/180.0, -89.65*pi/180.0, 74.21*pi/180.0]
Q12 = [162.97*pi/180.0, -57.38*pi/180.0, 95.30*pi/180.0, -126.56*pi/180.0, -89.65*pi/180.0, 74.25*pi/180.0]
Q20 = [170.71*pi/180.0, -45.96*pi/180.0, 97.31*pi/180.0, -140.12*pi/180.0, -89.48*pi/180.0, 81.87*pi/180.0]
Q21 = [170.52*pi/180.0, -51.35*pi/180.0, 95.76*pi/180.0, -133.19*pi/180.0, -89.48*pi/180.0, 81.74*pi/180.0]
Q22 = [170.40*pi/180.0, -57.07*pi/180.0, 94.19*pi/180.0, -125.88*pi/180.0, -89.48*pi/180.0, 81.68*pi/180.0]

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

Q = [ [Q00, Q01, Q02], \
      [Q10, Q11, Q12], \
      [Q20, Q21, Q22] ]

world_block = [[0], [0], [0]]

############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN

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

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, end_loc, end_height):
    ### Hint: Use the Q array to map out your towers by location and "height".
    global Q, world_block
    
    error = 0
    horiDist = abs(end_loc - start_loc)
    
    rospy.loginfo(f"Starting from Q[{start_loc}][{start_height}]")
    rospy.loginfo(f"Horizontal distance = {end_loc - start_loc}")
    
    move_arm(pub_cmd, loop_rate, Q[start_loc][2], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    rospy.Rate(2).sleep()
    
    if not digital_in_0:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        return error
    
    if (world_block[start_loc][0] == 1) and (world_block[end_loc][0] == 1):
        optHeight = start_height + 1
    elif (world_block[start_loc][0] == 2) and (world_block[end_loc][0] == 0):
        optHeight = start_height
    else:
        optHeight = 2
    
    if (world_block[start_loc][0] == 1) and (world_block[end_loc][0] == 0) and (horiDist == 1):
        move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)
    else:
        move_arm(pub_cmd, loop_rate, Q[start_loc][optHeight], 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, Q[end_loc][optHeight], 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)
        
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, Q[end_loc][2], 4.0, 4.0)

    # update world_block counters
    world_block[start_loc][0] -= 1
    world_block[end_loc][0] += 1
    
    rospy.loginfo(f"world_block = {world_block}")
    rospy.loginfo(f"Ending from Q[{end_loc}][{end_height}]")
    
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
    sub_gripper_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_input_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    location_list = [0, 1, 2]

    input_done = 0
    starting_location = -1

    while(not input_done):
        input_string = input("Enter towers starting location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            input_done = 1
            starting_location = 0
        elif (int(input_string) == 2):
            input_done = 1
            starting_location = 1
        elif (int(input_string) == 3):
            input_done = 1
            starting_location = 2
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
    
    location_list.remove(starting_location)
    
    input_done = 0
    ending_location = -1
    
    while(not input_done):
        input_string = input("Enter towers ending location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")
        
        if int(starting_location) == int(input_string) - 1:
            print("starting location can not be the same as ending location please try again")
            continue
            
        if(int(input_string) == 1):
            input_done = 1
            ending_location = 0
        elif (int(input_string) == 2):
            input_done = 1
            ending_location = 1
        elif (int(input_string) == 3):
            input_done = 1
            ending_location = 2
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    location_list.remove(ending_location)
    temp_location = location_list[0]

    world_block[starting_location][0] = 3


    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    if move_block(pub_command, loop_rate, starting_location, 2, ending_location, 0):
        return
    if move_block(pub_command, loop_rate, starting_location, 1, temp_location, 0):
        return
    if move_block(pub_command, loop_rate, ending_location, 0, temp_location, 1):
        return
    if move_block(pub_command, loop_rate, starting_location, 0, ending_location, 0):
        return
    if move_block(pub_command, loop_rate, temp_location, 1, starting_location, 0):
        return
    if move_block(pub_command, loop_rate, temp_location, 0, ending_location, 1):
        return
    if move_block(pub_command, loop_rate, starting_location, 0, ending_location, 2):
        return

    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
