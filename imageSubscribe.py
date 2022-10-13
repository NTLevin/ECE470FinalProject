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

# Define a callback for the Image message
def image_callback(img_msg):
    global QUIT, cv_image

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg)
    except CvBridgeError:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Flip the image 90deg
    cv_image = cv2.transpose(cv_image)
    cv_image = cv2.flip(cv_image, 1)
    cv2.imshow("cv_image", cv_image)
    if cv2.waitKey(1) == 27:
        QUIT = True
        cv2.destroyAllWindows()

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
# sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
sub_image = rospy.Subscriber("/cv_camera_node/image_raw", Image, image_callback)
QUIT = False

while (not rospy.is_shutdown() and not QUIT):
    pass