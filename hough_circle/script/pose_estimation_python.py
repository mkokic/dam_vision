#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

class pose_estimation():
    def __init__(self):
        self.node_name = 'pose_estimation_new'
        self.frame_id = rospy.get_param('camera_frame', 'camera_color_optical_frame')

        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
            
        self.sub = rospy.Subscriber("/hough_circle/circlepos", Vector3, self.callback, queue_size=1)
        
        
    def callback(self, msg):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        rospy.loginfo("the code goes right through here")
        radius = msg.z 
        x = msg.x
        y = msg.y
        cam_matrix = np.array([[619.016845703125, 0.0, 331.33929443359375], 
                                [0.0, 618.93115234375, 226.82150268554688], 
                                [0.0, 0.0, 1.0]], dtype=np.float32)
   
        dist = np.array([0.0,0.0,0.0,0.0,0.0], dtype=np.float32)

        self.qr_2d_projection = np.array([[0.0,radius,0.0],
                                          [radius,0.0,0.0],
                                          [0.0,-radius,0.0],
                                          [-radius,0.0,0.0]], dtype=np.float32)
                                

        self.img_pts = np.array([[x + radius,y],
                                [x, y + radius],
                                [x - radius, y],
                                [x, y - radius]], dtype=np.float32)                                   

        _, rvec, tvec = cv2.solvePnP(self.qr_2d_projection, self.img_pts, cam_matrix, dist)

        rospy.loginfo(rvec)
        rospy.loginfo(" dan tvecnya adalah ")
        rospy.loginfo(tvec)
  

    def cleanup(self):
        rospy.loginfo("pose_estimation node")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        pose_estimation()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Node shut down because of keyboard interrupt requested.")