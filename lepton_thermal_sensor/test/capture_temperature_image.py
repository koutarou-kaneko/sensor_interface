#!/usr/bin/env python

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/lepton_camera/temperature/image", Image, self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv_image)
        rospy.loginfo("temperature, max at [%d, %d]: %f, min at [%d, %d]: %f",
                      max_loc[0], max_loc[1], max_val, min_loc[0], min_loc[1], min_val);

if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('temperature_image_check', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


