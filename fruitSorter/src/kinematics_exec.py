#!/usr/bin/env python3

import sys
import copy
import time
import rospy

import numpy as np
from kinematics_header import *
from kinematics_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green and yellow blocks
xw_yw_G = []
xw_yw_Y = []
xw_yw_R = []

# xy0 = []
# xy1 = []
# zBlock = .03
# yPredicted_1 = []
# yPredicted1 = []
# blockCount = 0
# vy = []
# errory = 999
# tGuess = 0.5 #s
# invkGuess = []
# timeStart = 0
# timeFlag = 0
# accel = 4.0
# vel = 4.0

# timeStart = time.time() 
# xy0.append(xw_yw_R[0])

# if time.time() - timeStart >= 0.5 and timeFlag == 0:
#     xy1.append(xw_yw_R[0])
#     vy.append((xy1[0,1] - xy0[0,1]) / (time.time() - timeStart))
#     timeFlag = 1

# yPredicted_1 = xy1[0:1] + vy[0] * tGuess

# while errory > 0.005:
#     invkGuess = lab_invkNoMove(xy1[0,0], yPredicted_1, zBlock)

#     for i in range(0, len(invkGuess)):
#         if tGuess == 0.5:
#             tGuess = invkGuess[0] / (accel/2 - 2*vel)
#         elif invkGuess[0] / (accel/2 - 2*vel) > tGuess:
#             tGuess = invkGuess[0] / (accel/2 - 2*vel)

#     yPredicted1 = xy1[0,1] + vy[0] * tGuess
#     errory = abs(yPredicted1 - yPredicted_1)
#     yPredicted_1 = yPredicted1















# Any other global variable you want to define
# Hints: where to put the blocks?


# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


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


"""
Function to control the suction cup on/off
"""
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

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    timeInit = time.time()

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
            # print("TIMEFINAL", time.time() - timeInit)
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()
        # print("TIMEFINAL", time.time() - timeInit)

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    # print(target_xw_yw_zw)
    # target_tmp = [target_xw_yw_zw[0], target_xw_yw_zw[1],  + 0.2]

    # lab_invk(start_xw_yw_zw)
    print(f'Start = {start_xw_yw_zw}')
    start_xw_yw_zw = list(start_xw_yw_zw)
    tmp_start_xw_yw_zw = list(start_xw_yw_zw)
    start_xw_yw_zw.append(0.03)
    tmp_start_xw_yw_zw.append(0.07)
    
    target_xw_yw_zw = list(target_xw_yw_zw)
    tmp_target_xw_yw_zw = list(target_xw_yw_zw)
    target_xw_yw_zw.append(0.03)
    tmp_target_xw_yw_zw.append(0.07)

    # gripper(pub_cmd, loop_rate, suction_off)
    
    print(f'Start = {start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0}')
    print(f'End = {target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0}')
    
    print("move1")
    move_arm(pub_cmd, loop_rate, lab_invk(tmp_start_xw_yw_zw[0], tmp_start_xw_yw_zw[1], tmp_start_xw_yw_zw[2], 0), 4.0, 4.0)
    print("move2")
    move_arm(pub_cmd, loop_rate, lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0), 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    rospy.Rate(2).sleep()
    
    move_arm(pub_cmd, loop_rate, lab_invk(tmp_start_xw_yw_zw[0], tmp_start_xw_yw_zw[1], tmp_start_xw_yw_zw[2], 0), 4.0, 4.0)

    move_arm(pub_cmd, loop_rate, lab_invk(tmp_target_xw_yw_zw[0], tmp_target_xw_yw_zw[1], tmp_target_xw_yw_zw[2], 0), 4.0, 4.0)

    move_arm(pub_cmd, loop_rate, lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0), 4.0, 4.0)

    gripper(pub_cmd, loop_rate, suction_off)
    
    move_arm(pub_cmd, loop_rate, lab_invk(tmp_target_xw_yw_zw[0], tmp_target_xw_yw_zw[1], tmp_target_xw_yw_zw[2], 0), 4.0, 4.0)
    
    # gripper(pub_cmd, loop_rate, suction_on)
    
    # move_arm(pub_cmd, loop_rate, target_xw_yw_zw, 4.0, 4.0)
    
    # gripper(pub_cmd, loop_rate, suction_off)

    error = 0

    # ========================= Student's code ends here ===========================

    return error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_G # store found green blocks in this list
        global xw_yw_Y # store found yellow blocks in this list
        global xw_yw_R # store found red blocks in this list


        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y & xw_yw_R respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y or xw_yw_R.

        # Remember, xw_yw_G & xw_yw_Y & xw_yw_R are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        # xw_yw_G = blob_search(cv_image, "orange")
        xw_yw_Y = blob_search(cv_image, "yellow")
        xw_yw_G = blob_search(cv_image, "green")
        xw_yw_R = blob_search(cv_image, "red")


"""
Program run from here
"""

detect = True

def main():

    global go_away
    global xw_yw_Y
    global xw_yw_G
    global xw_yw_R
    global detect
    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    # time.sleep(5)

    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_G, xw_yw_Y to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """

    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    
    green_1_end_pos = [0.195, -0.16]
    yellow_1_end_pos = [0.25, -0.16]
    red_1_end_pos = [-.2, .6]
    operationRange = 0.15
    blockOffset = 0.03*3

    xy0 = [0, 0]
    xy1 = [0, 0]
    zBlock = .03
    yPredicted_1 = []
    yPredicted1 = []
    yPredicted = 0
    blockCount = 0
    vy = 0
    errory = 999
    tGuess = 0.5 #s
    invkGuess = []
    timeStart = 0
    timeFlag0 = 0
    timeFlag1 = 0
    moveFlag = 0



    while detect:
        greenPos = xw_yw_G
        yellowPos = xw_yw_Y
        redPos = xw_yw_R

        if greenPos != []:
            green_1_start_pos = greenPos[0]
            # # if abs(green_1_start_pos[1]) <= operationRange:
            # if (green_1_start_pos[1] > -0.15) and (green_1_start_pos[1] < 0.4):
            #     move_block(pub_command, loop_rate, green_1_start_pos, green_1_end_pos, vel, accel)
            # else:
            #     print(f'green_1_start_pos = {green_1_start_pos}')
        elif yellowPos != []:
            yellow_1_start_pos = yellowPos[0]
            # # if abs(yellow_1_start_pos[1]) <= operationRange:
            # if (yellow_1_start_pos[1] > -0.15) and (yellow_1_start_pos[1] < 0.4):
            #     move_block(pub_command, loop_rate, yellow_1_start_pos, yellow_1_end_pos, vel, accel)
            # else:
            #     print(f'yellow_1_start_pos = {yellow_1_start_pos}')
        elif redPos != []:
            red_1_start_pos = redPos[0]
            # print("redPos", redPos)
            if (red_1_start_pos[1] > -0.15) and (red_1_start_pos[1] < 0.4):
                red_1_start_pos = redPos[0]
                if timeFlag0 == 0:     
                    timeStart = time.time() 
                    timeFlag0 = 1
                    # xy0.append(np.array(redPos))
                    xy0[0] = redPos[0][0]
                    xy0[1] = redPos[0][1]
                    print("xy0", xy0)
                    # print("redpos", redPos[0][0])
                # print("redPos", redPos)
                # print(timeStart, time.time())
                # print(time.time() - timeStart)

                if time.time() - timeStart >= 0.25 and timeFlag1 == 0:
                    # xy1.append(np.array(red_1_start_pos))
                    xy1[0] = red_1_start_pos[0]
                    xy1[1] = red_1_start_pos[1]
                    print("here x0, x1",xy0, xy1)
                    print(xy1, xy0)
                    vy = ((xy1[1] - xy0[1]) / (time.time() - timeStart))
                    print("vy", vy)
                    print("x var", xy0, xy1, time.time(), timeStart)
                    timeFlag1 = 1
                
                if time.time() - timeStart >= 0.25 and timeFlag1 == 1:
                    yPredicted_1 = xy1[1] + vy * tGuess
                    # print("yPred1, errory", yPredicted_1, errory, tGuess)
                    # print("y0, y1, vy", xy0, xy1, vy)
                    angleDiff = []

                    while errory > 0.001:
                        print(time.time())
                        print("error", errory)
                        invkGuess = lab_invkNoMove(xy1[0], yPredicted_1, zBlock)
                        print("invKGuess", invkGuess)

                        if tGuess == 0.5:
                                    # tGuess = abs((invkGuess[0] - go_away[0])/ (accel/2 - 2*vel))
                                    tGuess = abs(4*(invkGuess[0] - go_away[0])/ (accel))
                
                                    
                        for i in range(0, len(invkGuess)):
                            if tGuess < 2:
                                for k in range(0, len(invkGuess)):
                                    angleDiff.append(invkGuess[k] - go_away[k])

                                print("angleDiff", angleDiff)
                                
                                print("initial", tGuess)
                            elif abs(4*(invkGuess[0] - go_away[0])/ (accel)) > tGuess:
                                # tGuess = abs(go_away[i]-invkGuess[i] / (accel/2 - 2*vel))
                                tGuess = abs(4*(invkGuess[i] - go_away[i])/ (accel))
                            else:   
                                if abs(((invkGuess[0] - go_away[0]) + 2 *vel**2 / accel - 2*vel /accel)/ (vel)) > tGuess:
                                    # tGuess = abs(go_away[i]-invkGuess[i] / (accel/2 - 2*vel))
                                    tGuess = abs(4*(invkGuess[i] - go_away[i])/ (accel))
                                    print("else")

                            tGuess_1 = tGuess
                            tGuess = 0.5
                            # print("guesses", i, tGuess)
                            # go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

                            
                            # print("tGuess", tGuess)

                        yPredicted = xy1[1] + vy * tGuess_1
                        print("y pred", yPredicted, yPredicted_1)
                        errory = abs(yPredicted - yPredicted_1)
                        yPredicted_1 = yPredicted
                        timeFlag1 = 0
                    if errory <= 0.005 and moveFlag == 0:
                        print("error less .005", errory)
                        moveFlag = 1

                    # if moveFlag ==1:
                    #     red_1_start_pos[1] = yPredicted
                if abs(yPredicted) <= operationRange and moveFlag ==1:
                    # if (red_1_start_pos[1] > -0.15) and (red_1_start_pos[1] < 0.4) and moveFlag ==1:
                    if (yPredicted > -0.15) and (yPredicted < 0.4) and moveFlag ==1:
                        print("yPred", yPredicted)
                        move_block(pub_command, loop_rate, [xy1[0], yPredicted + 0.14], red_1_end_pos, vel, accel)
                        xy0 = [0, 0]
                        xy1 = [0, 0]
                        zBlock = .03
                        yPredicted_1 = []
                        yPredicted = 0
                        blockCount = 0
                        vy = 0
                        errory = 999
                        tGuess = 0.5 #s
                        invkGuess = []
                        timeStart = 0
                        timeFlag0 = 0
                        timeFlag1 = 0
                        moveFlag = 0
                        redPos = []
                    else:
                        print('out of range')
                        yPredicted_1 = []
                        yPredicted = 0
                        blockCount = 0
                        vy = 0
                        errory = 999
                        tGuess = 0.5 #s
                        invkGuess = []
                        timeStart = 0
                        timeFlag0 = 0
                        timeFlag1 = 0
                        moveFlag = 0
                        redPos = []
                    #     print(f'red_1_start_pos = {red_1_start_pos}')
            else:
                print("no block")
                rospy.Rate(1).sleep()

    

    # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        detect = False
        pass
