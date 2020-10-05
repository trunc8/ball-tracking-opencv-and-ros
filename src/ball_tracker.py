#!/usr/bin/env python

# trunc8 did this
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Ball_Tracker:
  def __init__(self):
    rospy.Subscriber('ball_video', Image, self.trackerCallback)
    self.bridge = CvBridge()

  def trackerCallback(self, ros_msg):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(ros_msg, "bgr8")
    except CvBridgeError as e:
      print(e)
      return
    mask_img = self.get_masked_image(cv_img)
    _, contours, hierarchy = cv2.findContours(mask_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    index = -1
    thickness = 2
    color = (255,0,255)
    cv2.drawContours(cv_img, contours, index, color, thickness)
    cv2.imshow("Ball Video", cv_img)
    cv2.waitKey(3)

  def get_masked_image(self, rgb_img):
    hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
    # Tunable parameters: Lower and upper bounds of yellow color of ball
    yellowLower = (30, 50, 50)
    yellowUpper = (60, 255, 255)
    mask = cv2.inRange(hsv, yellowLower, yellowUpper)
    return mask

if __name__ == '__main__':
  try:
    rospy.init_node('ball_tracker')
    tracker_obj = Ball_Tracker()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("Node terminated.")
