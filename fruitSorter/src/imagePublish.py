#!/usr/bin/env python3
import rospy
from sensor_msgs import msg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cap = cv2.VideoCapture(0)
# Initialize the CvBridge class
bridge = CvBridge()

rospy.init_node('opencv_example', anonymous = True)
pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size=10)

while not rospy.is_shutdown():    
    ret, frame = cap.read()

    if not ret:
        break
    video_bridge = bridge.cv2_to_imgmsg(frame)
    # print(video_bridge)
    pub.publish(video_bridge)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) == 27:
        cap.release()
        cv2.destroyAllWindows()
        break