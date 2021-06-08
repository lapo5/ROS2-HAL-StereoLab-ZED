#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, PoseStamped
from cv_bridge import CvBridge
import threading

import sys
import pyzed.sl as sl

# Class definition of the calibration function
class SLAM_Zed_Node(Node):
	def __init__(self):
		super().__init__("slam_zed_node")
		self.get_logger().info("Calibration node is awake...")

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

		self.py_translation = sl.Translation()
		self.pose_data = sl.Transform()

		# Acquisition thread
		self.thread1 = threading.Thread(target=self.get_pose, daemon=True)
		self.thread1.start()

		# Publishers
		self.pose_pub = self.create_publisher(Pose, "/zed_camera/pose", 10)


	# This function save the current frame in a class attribute
	def get_pose(self):
		
		if self.zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = self.zed.get_position(self.camera_pose)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                self.rotation = camera_pose.get_rotation_vector()
                self.translation = camera_pose.get_translation(py_translation)
                self.pose_data = camera_pose.pose_data(sl.Transform())

                msg = PoseStamped()

				msg.header = Header()
				msg.header.stamp = self.get_clock().now().to_msg()
				msg.header.frame_id = "zed_cam"

				# Translation
				msg.pose.position.x = self.translation[0]
				msg.pose.position.y = self.translation[1]
				msg.pose.position.z = self.translation[2]

				rot = R.from_rotvec([self.rotation[0], self.rotation[1], self.rotation[2]])
				quat = rot.as_quat()

				# short-Rodrigues (angle-axis)
				msg.pose.orientation.x = quat[0]
				msg.pose.orientation.y = quat[1]
				msg.pose.orientation.z = quat[2]
				msg.pose.orientation.w = quat[3]

				# Publish the message
				self.pose_pub.publish(msg)

				t = geometry_msgs.msg.TransformStamped()

				t.header.stamp = self.get_clock().now().to_msg()
				t.header.frame_id = "world"
				t.child_frame_id = "Rover_CoM"
				t.transform.translation.x = msg.pose.position.x
				t.transform.translation.y = msg.pose.position.y
				t.transform.translation.z = msg.pose.position.z
				t.transform.rotation.x = quat[0]
				t.transform.rotation.y = quat[1]
				t.transform.rotation.z = quat[2]
				t.transform.rotation.w = quat[3]

				self.br.sendTransform(t)


				
	# This function stops/enable the acquisition stream
	def exit(self):
		self.zed.close()
		self.thread1.join()

	

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