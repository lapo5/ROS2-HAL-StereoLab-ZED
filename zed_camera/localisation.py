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
import math
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R

## 
# Basic class to handle the timestamp of the different sensors to know if it is a new sensors_data or an old one
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()

    ##
    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if (isinstance(sensor, sl.IMUData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_


# Class definition of the calibration function
class SLAM_Zed_Node(Node):
    def __init__(self):
        super().__init__("localisation_zed_node")
        self.get_logger().info("ZED Localisation node is awake...")

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

        self.bridge = CvBridge()
        self.frame = None

        self.init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.VGA,
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP,
                             camera_fps=100)
         
        self.zed = sl.Camera()
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().info(repr(status))
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

        self.pose_pub = self.create_publisher(PoseStamped, "/zed_camera/pose", qos_profile)
        self.odom_pub = self.create_publisher(Odometry, "/zed_camera/odom", qos_profile)
        self.imu_pub  = self.create_publisher(Imu, "/zed_camera/imu", qos_profile)


        qos_profile2 = QoSProfile(depth=1)
        qos_profile2.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile2.history = QoSHistoryPolicy.KEEP_LAST

        self.frame_pub = self.create_publisher(Image, "/zed_camera/raw_frame", qos_profile2)

        self.enable_publish_imu = True
        self.ts_handler = TimestampHandler()
        self.sensors_data = sl.SensorsData()

        self.do_slam = True
        self.enable_publish_pose_data = False

        self.grab_image = False
        self.mat = sl.Mat()

        self.frame = None
        self.bridge = CvBridge()

        self.stop_service = self.create_service(Empty, "/zed_camera/stop_localisation", self.stop_slam)
        self.image_toggle_service = self.create_service(Empty, "/zed_camera/toggle_image_grabbing", self.image_toggle)

        self.timer = self.create_timer(0.01, self.get_pose)

        self.timer_2 = self.create_timer(0.03, self.get_image)


    def get_image(self):

        if self.grab_image:
            err = self.zed.grab(self.runtime)
            if (err == sl.ERROR_CODE.SUCCESS) :
                self.zed.retrieve_image(self.mat, sl.VIEW.LEFT)
                self.frame_rbga = self.mat.get_data()
                self.frame = cv2.cvtColor(self.frame_rbga, cv2.COLOR_BGRA2GRAY)

                self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
                self.image_message.header = Header()
                self.image_message.header.stamp = self.get_clock().now().to_msg()
                self.image_message.header.frame_id = "zed_link"
                self.frame_pub.publish(self.image_message)

            else:
                self.get_logger().info("Error Grab Image")


    def stop_slam(self, request, response):
        self.do_slam = False
        self.grab_image = False

        return response


    def image_toggle(self, request, response):
 
        self.grab_image = not self.grab_image

        return response


    def get_pose(self):

        if self.do_slam:
            if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                tracking_state = self.zed.get_position(self.camera_pose)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.rotation = self.camera_pose.get_rotation_vector()
                    self.translation = self.camera_pose.get_translation(self.py_translation)
                    self.pose_data = sl.Pose()

                    if self.enable_publish_pose_data:
                        self.publish_pose_data()

                    self.publish_odom_data()
                else: 
                    self.get_logger().info("Tracking State: {0}".format(tracking_state))
                    
            else: 
                self.get_logger().info("Error in Grab Localisation ZED2 Data")

            if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
                # Check if the data has been updated since the last time
                # IMU is the sensor with the highest rate
                self.sensors_data.get_imu_data()
                self.imu_quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
                self.imu_linear_acceleration = self.sensors_data.get_imu_data().get_linear_acceleration()
                self.imu_angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()
            
                if self.enable_publish_imu:
                    self.publish_imu_data()


    def publish_imu_data(self):

        msg = Imu()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "zed_link"

        msg.orientation.x = self.imu_quaternion[0]
        msg.orientation.y = self.imu_quaternion[1]
        msg.orientation.z = self.imu_quaternion[2]
        msg.orientation.w = self.imu_quaternion[3]

        msg.orientation_covariance[0] = 0.5
        msg.orientation_covariance[4] = 0.5
        msg.orientation_covariance[8] = 0.5

        msg.angular_velocity.x = float(self.imu_angular_velocity[0])
        msg.angular_velocity.y = float(self.imu_angular_velocity[1])
        msg.angular_velocity.z = float(self.imu_angular_velocity[2])

        msg.angular_velocity_covariance[0] = 0.04
        msg.angular_velocity_covariance[4] = 0.04
        msg.angular_velocity_covariance[8] = 0.04

        msg.linear_acceleration.x = float(self.imu_linear_acceleration[0])
        msg.linear_acceleration.y = float(self.imu_linear_acceleration[1])
        msg.linear_acceleration.z = float(self.imu_linear_acceleration[2])

        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self.imu_pub.publish(msg)


    def publish_odom_data(self):

        msg = Odometry()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
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

        msg.pose.pose.position.x = float(world_to_rover[0, 3])
        msg.pose.pose.position.y = float(world_to_rover[1, 3])
        msg.pose.pose.position.z = float(world_to_rover[2, 3])

        #for i in range(0, 36):
        #    msg.pose.covariance[i] = self.pose_data.pose_covariance[i]

        for i in range(0, 6):
            msg.pose.covariance[i * 7] = 0.01

        for i in range(0, 6):
            msg.twist.covariance[i * 7] = 1e3

        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

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

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

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
        t.header.frame_id = "odom"
        t.child_frame_id = "zed_link"

        t.transform.translation.x = float(world_to_rover[0, 3])
        t.transform.translation.y = float(world_to_rover[1, 3])
        t.transform.translation.z = float(world_to_rover[2, 3])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)


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
        node.get_logger().info('ZED Loc Node stopped cleanly')
        node.exit()
    except BaseException:
        node.get_logger().info('Exception in ZED Loc Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        node.zed.close()
        node.destroy_node()
        rclpy.shutdown()
        
# Main
if __name__ == '__main__':
    main()
