#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import sys
import tf2_ros
import geometry_msgs


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

        # Acquisition thread
        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        # Publishers
        self.frame_pub = self.create_publisher(Image, "/zed_camera/raw_frame", 10)
        self.timer = self.create_timer(0.03, self.publish_frame)

        self.br = tf2_ros.TransformBroadcaster(self)


        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "Rover_CoM"
        static_transformStamped.child_frame_id = "ZED_Camera_Base"

        static_transformStamped.transform.translation.x =  0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0

        rot = R.from_euler('zyx', [0.0, 0.0, 0.0], degrees=True)
        quat = rot.as_quat()
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(static_transformStamped)


    # This function save the current frame in a class attribute
    def get_frame(self):

        err = self.cam.grab(self.runtime)
        if (err == sl.ERROR_CODE.SUCCESS) :
            self.cam.retrieve_image(self.mat, sl.VIEW.LEFT)
            self.frame = self.mat.get_data()


    # This function stops/enable the acquisition stream
    def exit(self):
        self.start_acquisition = False
        self.cam.disable_streaming()
        self.cam.close()
        self.thread1.join()



    # Publisher function
    def publish_frame(self):

        if len(self.frame is None or self.frame) == 0:
            return

        self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
        self.image_message.header = Header()
        self.image_message.header.stamp = self.get_clock().now().to_msg()
        self.image_message.header.frame_id = "ZED_Camera_Base"
        self.frame_pub.publish(self.image_message)



# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = ZedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
        node.exit()
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        node.destroy_node()
        rclpy.shutdown()


# Main
if __name__ == '__main__':
    main()