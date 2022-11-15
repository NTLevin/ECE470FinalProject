#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()
kernel = np.ones((3, 3),np.uint8)

# Define a callback for the Image message
def image_callback(img_msg):
    global QUIT, cv_image

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    except CvBridgeError:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Flip the image 90deg
    # cv_image = cv2.transpose(cv_image)
    # cv_image = cv2.flip(cv_image, 1)

    # convert to hsv colorspace
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    yelloContour = findContour(hsv, 'y')
    greenContour = findContour(hsv, 'g')
    redContour = findContour(hsv, 'r')

    cv2.drawContours(cv_image, yelloContour, -1, (255,0,0), 2)
    cv2.drawContours(cv_image, greenContour, -1, (255,0,0), 2)
    cv2.drawContours(cv_image, redContour, -1, (255,0,0), 2)

    cv2.imshow("cv_image", cv_image)
    # cv2.imshow("mask", mask)
    if cv2.waitKey(1) == 27:
        QUIT = True
        cv2.destroyAllWindows()

def findContour(hsv, color):
    global kernel
    if color == 'y' or color == 'Y':
        # lower bound and upper bound for Yellow color
        lower_bound = np.array([10, 100, 100])   
        upper_bound = np.array([50, 255, 255])
    elif color == 'g' or color == 'G':
        # lower bound and upper bound for Green color
        lower_bound = np.array([50, 20, 20])   
        upper_bound = np.array([100, 255, 255])
    elif color == 'r' or color == 'R':
        # lower bound and upper bound for Red color
        lower_bound = np.array([0, 100, 100])   
        upper_bound = np.array([0, 255, 255])
    else:
        return

    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Remove unnecessary noise from mask
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
# sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
# sub_image = rospy.Subscriber("/cv_camera_node/image_raw", Image, image_callback)
sub_image = rospy.Subscriber("/arm_sensor/camera/image_raw", Image, image_callback)
QUIT = False

while (not rospy.is_shutdown() and not QUIT):
    pass