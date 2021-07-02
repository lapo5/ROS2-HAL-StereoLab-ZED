#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import threading

import sys
import pyzed.sl as sl
import time
from scipy.spatial.transform import Rotation as R

# Class definition of the calibration function
class SLAM_Zed_Node(Node):
    def __init__(self):
        super().__init__("slam_zed_node")
        self.get_logger().info("ZED SLAM node is awake...")

        # Class attributes
        self.bridge = CvBridge()
        self.frame = None

        self.init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
         
        self.zed = sl.Camera()
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            sys.exit(1)

        self.tracking_params = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(self.tracking_params)

        self.runtime = sl.RuntimeParameters()
        self.camera_pose = sl.Pose()

        self.camera_info = self.zed.get_camera_information()

        '''
        self.stream = sl.StreamingParameters()
        self.stream.codec = sl.STREAMING_CODEC.H264
        self.stream.bitrate = 4000
        self.status = self.zed.enable_streaming(self.stream)
        if self.status != sl.ERROR_CODE.SUCCESS:
            print(repr(self.status))
            sys.exit(1)
        '''

        self.py_translation = sl.Translation()
        self.pose_data = sl.Transform()

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/zed_camera/pose", 0)
        self.odom_pub = self.create_publisher(Odometry, "/zed_camera/odom", 0)

        self.do_slam = True

        # Acquisition thread
        self.thread1 = threading.Thread(target=self.get_pose, daemon=True)
        self.thread1.start()

        '''
        self.mat = sl.Mat()

        self.frame = None
        self.bridge = CvBridge()

        self.acquire_frame = True


        # Acquisition thread
        self.thread2 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread2.start()

        # Publishers
        self.frame_pub = self.create_publisher(Image, "/zed_camera/raw_frame")
        self.timer = self.create_timer(0.03, self.publish_frame)
        '''



    # This function save the current frame in a class attribute
    def get_pose(self):

        while self.do_slam:
            if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                tracking_state = self.zed.get_position(self.camera_pose)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.rotation = self.camera_pose.get_rotation_vector()
                    self.translation = self.camera_pose.get_translation(self.py_translation)
                    self.pose_data = self.camera_pose.pose_data(sl.Transform())

                    msg = PoseStamped()

                    now = time.time()
                    msg.header = Header()
                    msg.header.stamp.sec = int(now)
                    msg.header.stamp.nanosec = int(now* 1e9) % 1000000000
                    msg.header.frame_id = "zed_cam"

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


                    odom_msg = Odometry()

                    now = time.time()
                    odom_msg.header = Header()
                    odom_msg.header.stamp.sec = int(now)
                    odom_msg.header.stamp.nanosec = int(now* 1e9) % 1000000000
                    odom_msg.header.frame_id = "zed_cam"

                    odom_msg.child_frame_id = "base_link"

                    # Translation
                    odom_msg.pose.pose.position.x = self.translation.get()[0]
                    odom_msg.pose.pose.position.y = self.translation.get()[1]
                    odom_msg.pose.pose.position.z = self.translation.get()[2]

                    rot = R.from_rotvec([self.rotation[0], self.rotation[1], self.rotation[2]])
                    quat = rot.as_quat()

                    # short-Rodrigues (angle-axis)
                    odom_msg.pose.pose.orientation.x = quat[0]
                    odom_msg.pose.pose.orientation.y = quat[1]
                    odom_msg.pose.pose.orientation.z = quat[2]
                    odom_msg.pose.pose.orientation.w = quat[3]

                    odom_msg.pose.covariance[0] = 0.1
                    odom_msg.pose.covariance[7] = 0.1
                    odom_msg.pose.covariance[14] = 0.1
                    odom_msg.pose.covariance[21] = 0.1
                    odom_msg.pose.covariance[28] = 0.1
                    odom_msg.pose.covariance[35] = 0.1

                    # Publish the message
                    self.odom_pub.publish(odom_msg)

    # This function stops/enable the acquisition stream
    def exit(self):
        self.do_slam = False
        self.thread1.join()

        #self.acquire_frame = False
        #self.thread2.join()
        
        self.zed.close()



    # This function save the current frame in a class attribute
    def get_frame(self):

        while self.acquire_frame:
            err = self.zed.grab(self.runtime)
            if (err == sl.ERROR_CODE.SUCCESS) :
                self.zed.retrieve_image(self.mat, sl.VIEW.LEFT)
                self.frame_rbga = self.mat.get_data()
                self.frame = cv2.cvtColor(self.frame_rbga, cv2.COLOR_BGRA2GRAY)



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
    node = SLAM_Zed_Node()
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
