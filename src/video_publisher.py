#!/usr/bin/env python

# trunc8 did this
import cv2
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Video_Publisher:
  def __init__(self):
    self.pub = rospy.Publisher('ball_video', Image, queue_size=10)
    self.rate = rospy.Rate(22)
    self.bridge = CvBridge()

  def publishVideo(self, filename):
    video_capture = cv2.VideoCapture(filename)
    while not rospy.is_shutdown():
      ret, frame = video_capture.read()
      try:
        ros_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
      except CvBridgeError as e:
        print(e)
        continue
      self.pub.publish(ros_msg)
      self.rate.sleep()

if __name__ == '__main__':
  try:
    rospy.init_node('video_pub')
    pub_obj = Video_Publisher()
    # Not including RosPack causes error if this script is rosrun from a different directory
    rospack = rospkg.RosPack()
    filename = rospack.get_path("ball_tracker") + "/video/tennis-ball-video.mp4"
    pub_obj.publishVideo(filename)
  except rospy.ROSInterruptException:
    rospy.loginfo("Node terminated.")
