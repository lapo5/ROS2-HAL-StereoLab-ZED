#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import threading

import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import sys
import tf2_ros
import geometry_msgs

# Class definition of the calibration function
class TFZedNode(Node):
    def __init__(self):
        super().__init__("tf_zed_camera_node")
        self.get_logger().info("TF ZED camera node is awake...")
        
        self.br = tf2_ros.TransformBroadcaster(self)
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "Rover_CoM"
        static_transformStamped.child_frame_id = "ZED_Camera_Base"

        static_transformStamped.transform.translation.x =  1.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.5

        rot = R.from_euler('zyx', [0.0, 0.0, 0.0], degrees=True)
        quat = rot.as_quat()
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(static_transformStamped)


        self.rotation_camera = np.eye(4, dtype=np.float32)
        self.rotation_camera[0, 0] = 0.0 
        self.rotation_camera[0, 1] = 0.0
        self.rotation_camera[0, 2] = -1.0 
        self.rotation_camera[1, 0] = -1.0
        self.rotation_camera[1, 1] = 0.0 
        self.rotation_camera[1, 2] = 0.0
        self.rotation_camera[2, 0] = 0.0
        self.rotation_camera[2, 1] = 1.0
        self.rotation_camera[2, 2] = 0.0 

        # Subscription
        self.pose_sub = self.create_subscription(PoseStamped, "/zed_camera/pose", self.callback_frame, 2)

    # This function store the received frame in a class attribute
    def callback_frame(self, msg):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "Rover_CoM"

        rot = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        cam_sight = np.eye(4, dtype=np.float32)
        cam_sight[0:3, 0:3] = rot.as_matrix()
        cam_sight[0, 3] = msg.pose.position.x
        cam_sight[1, 3] = msg.pose.position.y
        cam_sight[2, 3] = msg.pose.position.z

        world_to_rover = np.matmul(self.rotation_camera, cam_sight)

        rot_wTr = R.from_matrix(world_to_rover[0:3, 0:3])
        quat = rot.as_quat()

        t.transform.translation.x = float(world_to_rover[0, 3])
        t.transform.translation.y = float(world_to_rover[1, 3])
        t.transform.translation.z = float(world_to_rover[2, 3])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)

# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = TFZedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
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
