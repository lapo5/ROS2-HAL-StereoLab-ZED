#!/usr/bin/env pyhton3

##### Libraries
import rclpy
from rclpy.node import Node

import numpy, sys, gc


##### Interfaces
from sensor_msgs.msg import Imu


##### Controller class definition
class DecoratorNode(Node):
	def __init__(self):
		super().__init__("zed_imu_covariance_decorator_node")
		
		self.declare_parameter("learn_samples", 1000)
		self.learn_samples = self.get_parameter("learn_samples").value

		self.array = numpy.zeros((self.learn_samples, 6))
		self.variance = None
		self.bias = None
		self.count = 0

		self.publisher = self.create_publisher(Imu, "/zed_camera/imu_decorated", 10)
		self.get_logger().info("Waiting for Imu Messages on /zed_camera/imu...")
		self.subscriber = self.create_subscription(Imu, '/zed_camera/imu', self.learn_callback, 10)

	def learn_callback(self, msg):
		if self.count < self.learn_samples:
			self.array[self.count, 0] = msg.angular_velocity.x
			self.array[self.count, 1] = msg.angular_velocity.y
			self.array[self.count, 2] = msg.angular_velocity.z
			self.array[self.count, 3] = msg.linear_acceleration.x
			self.array[self.count, 4] = msg.linear_acceleration.y
			self.array[self.count, 5] = msg.linear_acceleration.z
			self.count = self.count + 1
			if self.count == self.learn_samples:
				self.subscriber.callback = self.empty_callback
				self.variance = numpy.var(self.array, axis=0)
				self.bias = numpy.mean(self.array, axis=0)
				self.get_logger().info(f"Angular Velocity variance: {self.variance[0:3]}")
				self.get_logger().info(f"Linear Acceleration variance: {self.variance[3:6]}")
				self.get_logger().info(f"Angular Velocity bias: {self.bias[0:3]}")
				# Linear acceleration bias makes no sense, gravity vector may be decomposed in many ways
				self.get_logger().info("Decorated data is being published to /zed_camera/imu_decorated...")
				del self.array
				gc.collect()
				self.subscriber.callback = self.decorate_callback


	def empty_callback(self,msg):
		pass

	def decorate_callback(self, msg):
		msg.angular_velocity.x = msg.angular_velocity.x - self.bias[0]
		msg.angular_velocity.y = msg.angular_velocity.y - self.bias[1]
		msg.angular_velocity.z = msg.angular_velocity.z - self.bias[2]
		msg.angular_velocity_covariance[0] = self.variance[0]
		msg.angular_velocity_covariance[4] = self.variance[1]
		msg.angular_velocity_covariance[8] = self.variance[2]
		msg.linear_acceleration_covariance[0] = self.variance[3]
		msg.linear_acceleration_covariance[4] = self.variance[4]
		msg.linear_acceleration_covariance[8] = self.variance[5]

		self.publisher.publish(msg)
		

##### Main function to loop
def main(args=None):
	rclpy.init(args=args)
	node = DecoratorNode()
	
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('Node imu_covariance_decorator_node stopped cleanly')
	except BaseException:
		print('Exception in Node imu_covariance_decorator_node:', file=sys.stderr)
		raise
	finally:
		node.destroy_node()
		rclpy.shutdown() 


##### Main Loop
if __name__ == "__main__":
	main()
