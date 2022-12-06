#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import apriltag

from fruitSorter.msg import aprilTagMsg

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

pub = rospy.Publisher('apriltag', aprilTagMsg)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a callback for the Image message
def image_callback(img_msg):
    global QUIT, cv_image

    try:
        # Convert ROS image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(img_msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    except CvBridgeError as e:
        print(e)

    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(cv_image)
    msg = aprilTagMsg()
    if len(results) == 1:
        print(results[0].tag_id)
        msg.id = int(results[0].tag_id)
    elif len(results) == 0:
        msg.id = int(0)
    pub.publish(msg)
    
    # rospy.Rate(0.2).sleep()

    cv2.imshow("cv_image", cv_image)
    # cv2.imshow("mask", mask)
    if cv2.waitKey(1) == 27:
        QUIT = True
        cv2.destroyAllWindows()

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
# sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
# sub_image = rospy.Subscriber("/cv_camera_node/image_raw", Image, image_callback)
sub_image = rospy.Subscriber("/arm_sensor/camera/image_raw", Image, image_callback)
QUIT = False

while (not rospy.is_shutdown() and not QUIT):
    pass