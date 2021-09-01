Skip to content
Search or jump to…
Pull requests
Issues
Marketplace
Explore
 
@lapo5 
lapo5
/
ROS2-HAL-StereoLab-ZED
2
1
0
Code
Issues
Pull requests
Actions
Projects
Wiki
Security
Insights
Settings
ROS2-HAL-StereoLab-ZED/zed_camera/camera_node.py /
@lapo5
lapo5 Fix
Latest commit fd86430 5 days ago
 History
 1 contributor
Executable File  139 lines (105 sloc)  3.8 KB
 
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
from zed_camera_interfaces.msg import CompressedImage

# Class definition of the calibration function
class ZedDecoderNode(Node):
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

        # Publishers
        
        self.cmd_vel = self.create_subscription(CompressedImage, "/zed_camera/decoded_frame", 
            self.callback_cmd_vel, 1)

        self.frame_pub = self.create_publisher(Image, "/zed_camera/decoded_frame")
        self.timer = self.create_timer(0.03, self.publish_frame)

        # Service: stop acquisition
        self.stop_service = self.create_service(Empty, "/zed_camera/stop_video_feed", self.stop_video_feed)


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
    node = ZedDecoderNode()
    try:
        while node.acquire_frame:
            rclpy.spin(node)
    except KeyboardInterrupt:
        print('Zed Decoder Node stopped cleanly')
        node.exit()
    except BaseException:
        print('Exception in ZED Decoder Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        
        node.thread1.join()
        node.cam.disable_streaming()
        node.cam.close()
        node.destroy_node()
        rclpy.shutdown()


# Main
if __name__ == '__main__':
    main()