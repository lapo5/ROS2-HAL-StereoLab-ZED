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

from zed_camera_interfaces.msg import CompressedImage

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
        self.init.camera_resolution = sl.RESOLUTION.HD720
        self.init.depth_mode = sl.DEPTH_MODE.NONE
        self.cam = sl.Camera()
        self.status = self.cam.open(self.init)
        if self.status != sl.ERROR_CODE.SUCCESS:
            print(repr(self.status))
            sys.exit(1)

        self.runtime = sl.RuntimeParameters()

        self.stream = sl.StreamingParameters()
        self.stream.codec = sl.STREAMING_CODEC.H264
        self.stream.bitrate = 8000
        self.stream.port = 30000 # Port used for sending the stream
        self.status = self.cam.enable_streaming(self.stream)

        if self.status != sl.ERROR_CODE.SUCCESS:
            print(repr(self.status))
            sys.exit(1)

        self.mat = sl.Mat()

        self.frame = None
        self.bridge = CvBridge()

        self.acquire_frame = True

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

        # Publishers
        self.frame_pub = self.create_publisher(CompressedImage, "/zed_camera/raw_frame", qos_profile)
        self.timer = self.create_timer(0.03, self.get_frame)

        # Service: stop acquisition
        self.stop_service = self.create_service(Empty, "/zed_camera/stop_video_feed", self.stop_video_feed)


    # This function stops/enable the acquisition stream
    def stop_video_feed(self, request, response):
        self.acquire_frame = False
        return response



    # This function save the current frame in a class attribute
    def get_frame(self):

        if self.acquire_frame:
            err = self.cam.grab(self.runtime)
            if (err == sl.ERROR_CODE.SUCCESS) :
                self.cam.retrieve_image(self.mat, sl.VIEW.LEFT)
                self.frame_rbga = self.mat.get_data()
                self.frame = cv2.cvtColor(self.frame_rbga, cv2.COLOR_BGRA2GRAY)

                print(self.frame.shape)

                self.get_logger().info("Sending Image")
                self.image_message = CompressedImage()
                self.image_message.data = []

                for i in range(0, 720):
                    for j in range(0, 1280):
                        self.image_message.data.insert(len(self.image_message.data), self.frame[i, j])

                self.frame_pub.publish(self.image_message)
                self.get_logger().info("Image Sent")

            else:
                self.get_logger().info("Error Grab Image")


    # This function stops/enable the acquisition stream
    def exit(self):
        self.acquire_frame = False



# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = ZedNode()
    try:
        while node.acquire_frame:
            rclpy.spin(node)
    except KeyboardInterrupt:
        print('ZED Camera Node stopped cleanly')
        node.exit()
    except BaseException:
        print('Exception in ZED Camera Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        
        node.cam.disable_streaming()
        node.cam.close()
        node.destroy_node()
        rclpy.shutdown()


# Main
if __name__ == '__main__':
    main()
