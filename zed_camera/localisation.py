#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import threading

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import sys
import pyzed.sl as sl
import time
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R

# Class definition of the calibration function
class SLAM_Zed_Node(Node):
    def __init__(self):
        super().__init__("slam_zed_node")
        self.get_logger().info("ZED SLAM node is awake...")

        self.br = tf2_ros.TransformBroadcaster(self)

        self.rotation_camera = np.eye(4, dtype=np.float32)
        self.rotation_camera[0, 0] = 0.0 
        self.rotation_camera[0, 1] = 1.0
        self.rotation_camera[0, 2] = 0.0
        self.rotation_camera[1, 0] = -1.0
        self.rotation_camera[1, 1] = 0.0
        self.rotation_camera[1, 2] = 0.0
        self.rotation_camera[2, 0] = 0.0
        self.rotation_camera[2, 1] = 0.0
        self.rotation_camera[2, 2] = 1.0 

        # Class attributes
        self.bridge = CvBridge()
        self.frame = None

        self.init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP)
         
        self.zed = sl.Camera()
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            sys.exit(1)

        self.tracking_params = sl.PositionalTrackingParameters( _enable_memory=True, 
                                                                _enable_pose_smoothing=False, 
                                                                _enable_imu_fusion=True)

        self.zed.enable_positional_tracking(self.tracking_params)

        self.runtime = sl.RuntimeParameters()
        self.camera_pose = sl.Pose()

        self.camera_info = self.zed.get_camera_information()

        self.py_translation = sl.Translation()
        self.pose_data = sl.Transform()


        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST


        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/zed_camera/pose", qos_profile)
        self.odom_pub = self.create_publisher(Odometry, "/zed_camera/odom", qos_profile)
        self.imu_pub  = self.create_publisher(Imu, "/zed_camera/imu", qos_profile)

        self.enable_publish_imu = True
        self.ts_handler = TimestampHandler()
        self.sensor_data = sl.SensorsData()

        self.do_slam = True
        self.enable_publish_pose_data = False

        # Acquisition thread
        self.thread1 = threading.Thread(target=self.get_pose, daemon=True)
        self.thread1.start()

        # Service: stop acquisition
        self.stop_service = self.create_service(Empty, "/zed_camera/stop_slam", self.stop_slam)



    # This function stops/enable the acquisition stream
    def stop_slam(self, request, response):
        self.do_slam = False

        return response

    # This function save the current frame in a class attribute
    def get_pose(self):

        while self.do_slam:
            if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                tracking_state = self.zed.get_position(self.camera_pose)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.rotation = self.camera_pose.get_rotation_vector()
                    self.translation = self.camera_pose.get_translation(self.py_translation)
                    self.pose_data = sl.Pose()

                    if self.ts_handler.is_new(sensors_data.get_imu_data()):
                        self.quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
                        print("IMU Orientation: {}".format(self.quaternion))
                        self.linear_acceleration = self.sensors_data.get_imu_data().get_linear_acceleration()
                        print("IMU Acceleration: {} [m/sec^2]".format(self.linear_acceleration))
                        self.angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()
                        print("IMU Angular Velocity: {} [deg/sec]".format(self.angular_velocity))

                        if self.enable_publish_imu:
                            self.publish_imu_data()


                    if self.enable_publish_pose_data:
                        self.publish_pose_data()

                    self.publish_odom_data()

    def publish_imu_data(self):

        msg = Imu()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "zed_link"

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0

        msg.orientation_covariance[0] = 1e3
        msg.orientation_covariance[4] = 1e3
        msg.orientation_covariance[8] = 1e3

        msg.angular_velocity.x = float(self.rotvel[0])
        msg.angular_velocity.y = float(self.rotvel[1])
        msg.angular_velocity.z = float(self.rotvel[2])

        msg.angular_velocity_covariance[0] = 0.0004
        msg.angular_velocity_covariance[4] = 0.0004
        msg.angular_velocity_covariance[8] = 0.0004

        msg.linear_acceleration.x = float(self.linacc[0])
        msg.linear_acceleration.y = float(self.linacc[1])
        msg.linear_acceleration.z = float(self.linacc[2])

        msg.linear_acceleration_covariance[0] = 0.0004
        msg.linear_acceleration_covariance[4] = 0.0004
        msg.linear_acceleration_covariance[8] = 0.0004

        self.imu_pub.publish(msg)

    def publish_odom_data(self):

        msg = Odometry()

        now = time.time()
        msg.header = Header()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int(now* 1e9) % 1000000000
        msg.header.frame_id = "odom"
        msg.child_frame_id = "zed_link"

        rot = R.from_rotvec([self.rotation[0], self.rotation[1], self.rotation[2]])
        quat = rot.as_quat()

        cam_sight = np.eye(4, dtype=np.float32)
        cam_sight[0:3, 0:3] = rot.as_matrix()
        cam_sight[0, 3] = self.translation.get()[0]
        cam_sight[1, 3] = self.translation.get()[1]
        cam_sight[2, 3] = self.translation.get()[2]

        world_to_rover = np.matmul(self.rotation_camera, cam_sight)

        rot_wTr = R.from_matrix(world_to_rover[0:3, 0:3])
        quat = rot.as_quat()

        # Translation
        msg.pose.pose.position.x = float(world_to_rover[0, 3])
        msg.pose.pose.position.y = float(world_to_rover[1, 3])
        msg.pose.pose.position.z = float(world_to_rover[2, 3])

        for i in range(0, 36):
            msg.pose.covariance[i] = self.pose_data.pose_covariance[i]

        for i in range(0, 6):
            msg.twist.covariance[i * 7] = 1e3

        # short-Rodrigues (angle-axis)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Publish the message
        self.odom_pub.publish(msg)


    def publish_pose_data(self):

        msg = PoseStamped()

        now = time.time()
        msg.header = Header()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int(now* 1e9) % 1000000000
        msg.header.frame_id = "zed_link"

        # Translation
        msg.pose.position.x = self.translation.get()[0]
        msg.pose.position.y = self.translation.get()[1]
        msg.pose.position.z = self.translation.get()[2]

        rot = R.from_rotvec([self.rotation[0], self.rotation[1], self.rotation[2]])
        quat = rot.as_quat()

        # short-Rodrigues (angle-axis)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        # Publish the message
        self.pose_pub.publish(msg)

        rot = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        cam_sight = np.eye(4, dtype=np.float32)
        cam_sight[0:3, 0:3] = rot.as_matrix()
        cam_sight[0, 3] = msg.pose.position.x
        cam_sight[1, 3] = msg.pose.position.y
        cam_sight[2, 3] = msg.pose.position.z

        world_to_rover = np.matmul(self.rotation_camera, cam_sight)

        rot_wTr = R.from_matrix(world_to_rover[0:3, 0:3])
        quat = rot.as_quat()

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "zed_odom"
        t.child_frame_id = "zed_link"

        t.transform.translation.x = float(world_to_rover[0, 3])
        t.transform.translation.y = float(world_to_rover[1, 3])
        t.transform.translation.z = float(world_to_rover[2, 3])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)

    # This function stops/enable the acquisition stream
    def exit(self):
        self.do_slam = False

# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = SLAM_Zed_Node()
    try:
        while node.do_slam:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('ZED SLAM Node stopped cleanly')
        node.exit()
    except BaseException:
        print('Exception in ZED SLAM Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        node.thread1.join()
        node.zed.close()
        node.destroy_node()
        rclpy.shutdown()
        
# Main
if __name__ == '__main__':
    main()
