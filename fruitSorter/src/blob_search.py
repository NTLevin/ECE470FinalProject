#!/usr/bin/env python

import cv2
import numpy as np
import math
from scipy.linalg import expm

# ========================= Student's code starts here =========================

# Params for camera calibration
# theta = 0.0
theta = 0
beta = 766
tx = 0.29
ty = 0.11

# 0.16 0.235

# Function that converts image coord to world coord
def IMG2W(col, row):
    global beta, theta, tx, ty
    
    px = 240
    py = 320
    
    x_c = (row - px) / beta
    y_c = (col - py) / beta
    
    camera_pos = np.zeros((2, 1))
    camera_pos[0] = x_c
    camera_pos[1] = y_c
    
    R = np.array([[math.cos(theta), -math.sin(theta)],
                  [math.sin(theta), math.cos(theta)]])

    world_x = math.cos(theta) * x_c - math.sin(theta) * y_c + tx
    world_y = math.sin(theta) * x_c + math.cos(theta) * y_c + ty
    return world_x, world_y

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):
    global beta, theta
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    # print(f'color = {color}')
    if color == "red":
        kpColor = (0, 0, 255)
        lower = (0, 100, 100)    # red lower
        upper = (0, 255, 255)   # red upper
    elif color == "yellow":
        kpColor = (255, 0, 255)
        lower = (10, 100, 100)    # yellow lower
        upper = (50, 255, 255)   # yellow upper
    elif color == "green":
        kpColor = (0, 255, 0)
        lower = (50, 20, 20)     # light green lower
        upper = (100, 255, 255)
    else:
        return

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    # print(f'num_blobs = {num_blobs}')
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0], keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, kpColor,
                                 flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================
    # print(f'blob_image_center = {blob_image_center}')

    xw_yw = []

    if len(blob_image_center) == 1:
        pt = blob_image_center[0]
        # print(f'blob_image_center = {pt}')
        # Convert image coordinates to global world coordinate using IM2W() function
        xw_yw.append(IMG2W(pt[0], pt[1]))

    # cv2.circle(im_with_keypoints, (320, 240), 1, (255, 0, 0), 1)
    
    # cv2.namedWindow("Camera View")
    # cv2.imshow("Camera View", image_raw)
    # cv2.namedWindow("Mask View " + color)
    # cv2.imshow("Mask View " + color, mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
