#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import threading

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
        self.stream.bitrate = 4000
        self.status = self.cam.enable_streaming(self.stream)
        if self.status != sl.ERROR_CODE.SUCCESS:
            print(repr(self.status))
            sys.exit(1)

        self.mat = sl.Mat()

        self.frame = None
        self.bridge = CvBridge()

        self.acquire_frame = True

        # Acquisition thread
        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        # Publishers
        self.frame_pub = self.create_publisher(Image, "/zed_camera/raw_frame")
        self.timer = self.create_timer(0.03, self.publish_frame)



    # This function save the current frame in a class attribute
    def get_frame(self):

        while self.acquire_frame:
            err = self.cam.grab(self.runtime)
            if (err == sl.ERROR_CODE.SUCCESS) :
                self.cam.retrieve_image(self.mat, sl.VIEW.LEFT)
                self.frame_rbga = self.mat.get_data()
                self.frame = cv2.cvtColor(self.frame_rbga, cv2.COLOR_BGRA2GRAY)


    # This function stops/enable the acquisition stream
    def exit(self):
        self.acquire_frame = False
        self.thread1.join()
        self.cam.disable_streaming()
        self.cam.close()



    # Publisher function
    def publish_frame(self):

        if self.frame is None or len(self.frame) == 0:
            return

        self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
        self.image_message.header = Header()
        now = time.time()
        self.image_message.header = Header()
        self.image_message.header.stamp.sec = int(now)
        self.image_message.header.stamp.nanosec = int(now* 1e9) % 1000000000
        self.image_message.header.frame_id = "ZED_Camera_Base"
        self.frame_pub.publish(self.image_message)



# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = ZedNode()
    try:
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
        node.destroy_node()
        rclpy.shutdown()


# Main
if __name__ == '__main__':
    main()
