#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from pymba import *
from sensor_msgs.msg import Image
from allied_vision_camera_interfaces.srv import CameraState
from cv_bridge import CvBridge
import threading



# Class definition of the calibration function
class CalibrationNode(Node):
	def __init__(self):
		super().__init__("av_camera_node")
		self.get_logger().info("Calibration node is awake...")
		
		# Parameters declaration
		self.declare_parameter("cam_id", 0)

		# Class attributes
		self.cam_id = self.get_parameter("cam_id").value
		self.cam_obj = None
		self.bridge = CvBridge()
		self.frame = []
		self.start_acquisition = True

		# Acquisition thread
		self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
		self.thread1.start()

		# Publishers
		self.frame_pub = self.create_publisher(Image, "frame", 10)
		self.timer = self.create_timer(0.03, self.publish_frame)

		# Service: stop acquisition
		self.stop_service = self.create_service(CameraState, "cam_state", self.acquisition_service)

	# This function stops/enable the acquisition stream
	def acquisition_service(self, request, response):
		self.start_acquisition = request.command_state
		response.cam_state = self.start_acquisition
		return response


	# This function save the current frame in a class attribute
	def get_frame(self):
		
		with Vimba() as vimba:
			
			# Open the cam and set the mode
			self.cam_obj = vimba.camera(self.cam_id)
			self.cam_obj.open()
			self.cam_obj.arm("SingleFrame")
			self.get_logger().info("Fame acquisition has started.")
			
			while self.start_acquisition:
				current_frame = self.cam_obj.acquire_frame()
				self.frame = current_frame.buffer_data_numpy()

			self.cam_obj.disarm()
			self.cam_obj.close()
				

	# Publisher function
	def publish_frame(self):
		
		if len(self.frame) == 0:
			return

		self.frame_pub.publish(self.bridge.cv2_to_imgmsg(self.frame))
		self.get_logger().info("Frame published.")



# Main loop function
def main(args=None):

	rclpy.init(args=args)
	node = CalibrationNode()
	rclpy.spin(node)
	rclpy.shutdown()


# Main
if __name__ == '__main__':
	main()