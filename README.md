# Ball Tracking Using OpenCV And ROS

### Demo
![Ball Tracking Demo](Ball_Tracking_Demo.gif)

### Steps to Replicate
1. `git clone https://github.com/trunc8/Ball_Tracking_Using_OpenCV_And_ROS.git ball_tracking` in the *src* folder of your *catkin_ws* (or equivalent ROS workspace).
2. `roscore` in Tab 1
3. `rosrun ball_tracking video_publisher.py` in Tab 2
4. `rosrun ball_tracking ball_tracker.py` in Tab 3

### What This Does
- `video_publisher.py` reads video file using OpenCV, converts the frames to ROS Image Message using CvBridge and publishes to topic '/ball_video'
- `ball_tracker.py` subscribes to the ROS Image topic, suitably converts to OpenCV image and performs contour tracking on the ball

### Author

* **Siddharth Saha** - [trunc8](https://github.com/trunc8)

<p align='center'>Created with :heart: by <a href="https://www.linkedin.com/in/sahasiddharth611/">Siddharth</a></p>
