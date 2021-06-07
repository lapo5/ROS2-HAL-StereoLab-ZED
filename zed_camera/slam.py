#!/usr/bin/env python3

# Libraries
import cv2
from cv2 import aruco
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import threading
from sensor_msgs.msg import Image
from interfaces.msg import Pose
from interfaces.srv import CameraState
from functools import partial
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# Paths
CALIB_PATH = "./src/camera/camera/calib_params.json"
MARKER_SIDE = 0.1 # meters



# Class definition fo the estimator
class ArucoPoseNode(Node):
	def __init__(self):
		super().__init__("aruco_pose_estimator")
		self.get_logger().info("Marker estimator node is awake...")

		# Class attributes
		self.cam_params = dict()
		self.get_logger().info("Uploading intrinsic parameters from " + CALIB_PATH)
		self.get_cam_parameters()
		self.get_logger().info("Parameters successfully uploaded.")
		self.frame = []
		self.marker_pose = []
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
		self.aruco_params = aruco.DetectorParameters_create()
		self.bridge = CvBridge()

		# Subscription
		self.frame_sub = self.create_subscription(Image, "frame", self.callback_frame, 10)

		# Publishers
		self.pose_pub = self.create_publisher(Pose, "raw_pose", 10)
		self.pose_timer = self.create_timer(0.03, self.publish_pose)

		# Estimation process
		self.thread1 = threading.Thread(target=self.estimate_pose, daemon=True)
		self.thread1.start()


	# Destructor function: call the stop service and disarm the camera regularly
	def __del__(self):
		self.callback_stop_service(False)


	# This function is a client which asks the camera to shutdown when the node is killed
	def  callback_stop_service(self, stop_flag):
		client = self.create_client(CameraState, "cam_state")
		while not client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for server response...")

		request = CameraState.Request()
		request.command_state = stop_flag

		future = client.call_async(request)
		future.add_done_callback(partial(self.callback_call_stop_service, stop_flag=stop_flag))

	# This function is a callback to the client future
	def callback_call_stop_service(self, future, stop_flag):
		try:
			response = future.result()
			self.get_logger().info("Camera state has been set: " + str(response.cam_state))
		except Exception as e:
			self.get_logger().info("Service call failed %r" %(e,))



	# This function publish the pose information from each frame
	def publish_pose(self):
		if len(self.marker_pose) != 0:
			
			msg = Pose()

			# If the marker is in view
			if self.marker_pose[2]:
				

				# Translation
				msg.x = self.marker_pose[0][0][0][0]
				msg.y = self.marker_pose[0][0][0][1]
				msg.z = self.marker_pose[0][0][0][2]

				# short-Rodrigues (angle-axis)
				msg.r1 = self.marker_pose[1][0][0][0]
				msg.r2 = self.marker_pose[1][0][0][1]
				msg.r3 = self.marker_pose[1][0][0][2]

				# in view
				msg.in_view = True

			else:

				msg.in_view = False

			# Publish the message
			self.pose_pub.publish(msg)
		

	# This function upload from JSON the intrinsic camera parameters k_mtx and dist_coeff
	def get_cam_parameters(self):
		with open(CALIB_PATH, "r") as readfile:
			self.cam_params = json.load(readfile)

		self.cam_params["mtx"] = np.array(self.cam_params["mtx"], dtype=float).reshape(3, 3)
		self.cam_params["dist"] = np.array(self.cam_params["dist"], dtype=float)



	# This function store the received frame in a class attribute
	def callback_frame(self, msg):
		frame = self.bridge.imgmsg_to_cv2(msg)
		if len(frame.shape) == 3:
			self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		else:
			self.frame = frame

		self.get_logger().info("Frame received.")



	# This function detect and estimate the marker pose wrt the camera frame
	def estimate_pose(self):

		while True:

			# If the 1st frame has not been received yet, return
			while len(self.frame) != 0:

				corners, ids, _ = aruco.detectMarkers(self.frame, self.aruco_dict, parameters = self.aruco_params)

				# If there are no ids, jump in order not to make the code crashes
				if np.all(ids != None):

					# Pose estimation for each marker
					rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIDE, 
						self.cam_params["mtx"], self.cam_params["dist"])

					# (!!!) This line makes sense only if we use a single marker detection
					self.marker_pose = [tvec, rvec, True]


					# Draw the axis on the aruco markers
					for i in range(0, ids.size):
						aruco.drawAxis(self.frame, self.cam_params["mtx"], self.cam_params["dist"], rvec[i], tvec[i], 0.1)

					# Draw a square on the markers perimeter
					aruco.drawDetectedMarkers(self.frame, corners)

				else:
					self.marker_pose = [0, 0, False]

				# Show the frame
				cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
				cv2.imshow("frame", self.frame)
				cv2.waitKey(1)
	


# Main loop function
def main(args=None):
	rclpy.init(args=args)
	node = ArucoPoseNode()
	rclpy.spin(node)
	rclpy.shutdown()


# Main
if __name__ == '__main__':
	main()