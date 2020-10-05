#!/usr/bin/env python

# trunc8 did this
import numpy as np
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
    # One method of drawing contours
    # index = -1
    # thickness = 2
    # color = (255,0,255)
    # cv2.drawContours(cv_img, contours, index, color, thickness)
    # cv2.imshow("Ball Video", cv_img)
    # cv2.waitKey(3)
    self.draw_ball_contours(cv_img, contours)

  def get_masked_image(self, rgb_img):
    hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
    # Tunable parameters: Lower and upper bounds of yellow color of ball
    yellowLower = (30, 50, 50)
    yellowUpper = (60, 255, 255)
    mask = cv2.inRange(hsv, yellowLower, yellowUpper)
    return mask

  def draw_ball_contours(self, rgb_img, contours):
    black_img = np.zeros(rgb_img.shape, 'uint8')
    for c in contours:
      area = cv2.contourArea(c)
      ((x,y), radius) = cv2.minEnclosingCircle(c)
      if (area > 5000):
        cv2.drawContours(rgb_img, [c], -1, (255,0,255), 2)
        cx, cy = self.get_contour_center(c)
        cv2.circle(rgb_img, (cx,cy), (int)(radius), (0,255,255), 3)
        cv2.circle(black_img, (cx,cy), (int)(radius), (0,255,255), 3)
        cv2.circle(black_img, (cx,cy), 5, (150,0,255), -1)
      # print("Area: ", area)
    cv2.imshow("Ball Tracking", rgb_img)
    cv2.imshow("Black Background Tracking", black_img)
    cv2.waitKey(3)

  def get_contour_center(self, contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    return cx, cy

if __name__ == '__main__':
  try:
    rospy.init_node('ball_tracker')
    tracker_obj = Ball_Tracker()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("Node terminated.")
