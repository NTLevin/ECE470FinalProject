#!/usr/bin/env python3

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
# theta = np.deg2rad(8.1)
theta = np.deg2rad(-1.1)
beta = 765.5509406052513
# beta = 900
# measurement to (320,240) in m -> (tx, ty)
# tx = 0.215
# ty = 0.105
tx = (240 - 76) / beta
ty = (320 - 235) / beta


# Function that converts image coord to world coord
def IMG2W(col, row):

    # print(blob_image_center)    
    # Beta = np.sqrt((blob_image_center[0][0]-blob_image_center[1][0])**2 + (blob_image_center[0][1]-blob_image_center[1][1])**2)/.10 
    # print(Beta)
    Tx = row
    Ty = col
    # print(Tx, Ty)
    # print(blob_image_center[0])

    xc = (row - 240) / beta + tx
    yc = (col - 320) / beta + ty
    xW = xc * np.cos(theta) - yc * np.sin(theta)
    yW = yc * np.cos(theta) + xc * np.sin(theta)
    # print(xW, yW)
    # print(row,)
    # print(Tx, Ty)
    # print(xW, yW)
    return xW, yW

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):
    
    
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False
    # params.maxConvexity = .95

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    
    # color = "orange"
    # color = "green"
    if color == "blue":
        lower = (110,50,50)     # blue lower
        upper = (130,255,255)   # blue upper
        
    elif color == "orange":
        lower = (5,100,100)     # orange lower
        upper = (20,255,255)   # orange upper

    # elif color == "orange":
    #     lower = (10,150,150)     # orange lower
    #     upper = (24,255,255)   # orange upper
    elif color == "yellow":
        lower = (20,100,100)     # yellow lower
        upper = (30,255,255)   # yellow upper

    elif color == "green":
        # lower = (45,50,50)     # light green lower
        # upper = (65,255,255)   # light green upper
        lower = (35,45,45)     # light green lower
        upper = (70,255,255)   # light green upper

    else:
        print("Incorrect color input")

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================
    
    
    # theta = np.rad2deg(np.arccos(((-blob_image_center[0][0] + blob_image_center[1][0])/beta)/0.1))
    # print(theta)
        
     # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints,mask_image)
    
    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)
    # print(xw_yw)
    return xw_yw
