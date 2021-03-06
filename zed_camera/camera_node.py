#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import threading

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import sys
import geometry_msgs
import time

"""
    Open the camera and start streaming images using H264 codec
"""
import sys
import pyzed.sl as sl

# Class definition of the calibration function
class ZedNode(Node):
    def __init__(self):
        super().__init__("zed_camera_node")
        self.get_logger().info("ZED camera node is awake...")

        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.VGA
        self.init.depth_mode = sl.DEPTH_MODE.NONE
        self.init.camera_fps = 100
        self.cam = sl.Camera()
        self.status = self.cam.open(self.init)
        if self.status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().info(repr(self.status))
            sys.exit(1)

        self.runtime = sl.RuntimeParameters()

        self.mat = sl.Mat()

        self.frame = None
        self.bridge = CvBridge()

        self.acquire_frame = True

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        self.frame_pub = self.create_publisher(Image, "/zed_camera/raw_frame", qos_profile)
        self.timer = self.create_timer(0.01, self.get_frame)

        self.stop_service = self.create_service(Empty, "/zed_camera/stop_video_feed", self.stop_video_feed)


    def stop_video_feed(self, request, response):
        self.acquire_frame = False
        return response


    def get_frame(self):

        if self.acquire_frame:
            err = self.cam.grab(self.runtime)
            if (err == sl.ERROR_CODE.SUCCESS) :
                self.cam.retrieve_image(self.mat, sl.VIEW.LEFT)
                self.frame_rbga = self.mat.get_data()
                self.frame = cv2.cvtColor(self.frame_rbga, cv2.COLOR_BGRA2GRAY)

                self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
                self.image_message.header = Header()
                self.image_message.header.stamp = self.get_clock().now().to_msg()
                self.image_message.header.frame_id = "zed_link"
                self.frame_pub.publish(self.image_message)

            else:
                self.get_logger().info("Error Grab Image")


    def exit(self):
        self.acquire_frame = False



def main(args=None):

    rclpy.init(args=args)
    node = ZedNode()
    try:
        while node.acquire_frame:
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ZED Camera Node stopped cleanly')
        node.exit()
    except BaseException:
        node.get_logger().info('Exception in ZED Camera Node:', file=sys.stderr)
        raise
    finally:
        node.cam.disable_streaming()
        node.cam.close()
        node.destroy_node()
        rclpy.shutdown()


# Main
if __name__ == '__main__':
    main()
