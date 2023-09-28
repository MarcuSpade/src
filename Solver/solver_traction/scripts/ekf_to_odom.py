#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import tf2_ros

class ekf_fusion():
	def __init__(self,name):
		self.name=name
		self.rospy=rospy
		self.rospy.init_node(self.name,anonymous=True)
		self.rospy.loginfo("[%s] Starting Node ",self.name)
		self.initVariables()
		self.initPublishers()
		self.initSubscribers()	
		self.main()

	def initVariables(self):
		self.odomMsg = Odometry()
		self.imuMsg = Imu()
		self.newOdom = Odometry()
		self.ekf_odom_msg = PoseWithCovarianceStamped()
		return
	
	def initPublishers(self):
		self.ekf_odom_pub = self.rospy.Publisher('/odom', Odometry, queue_size=10)
		return

	def initSubscribers(self):
		self.sub_odom =  self.rospy.Subscriber("/odom_raw", Odometry ,self.odom_callback)
		self.sub_imu =  self.rospy.Subscriber("/imu/data", Imu ,self.imu_callback)
		self.ekf_sub = self.rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.ekf_callback)
		return


	def odom_callback(self,data):
		self.odomMsg = data
		return

	def imu_callback(self,data):
		self.imuMsg = data
		return

	def ekf_callback(self,data):
		# print(data)

		br = tf2_ros.TransformBroadcaster()
		t = TransformStamped()	

		x = data.pose.pose.position.x
		y = data.pose.pose.position.y
		z = data.pose.pose.position.z

		qw = data.pose.pose.orientation.w
		qx = data.pose.pose.orientation.x
		qy = data.pose.pose.orientation.y
		qz = data.pose.pose.orientation.z

		## construct tf
		#t.header.frame_id = "odom" 
		#t.header.stamp = rospy.Time.now()
		#t.child_frame_id = "base_link"	#"base_footprint"	#"base_link"  # base_link_ekf
		#t.transform.translation.x = x
		#t.transform.translation.y = y
		#t.transform.translation.z = z

		#t.transform.rotation.x = qx
		#t.transform.rotation.y = qy
		#t.transform.rotation.z = qz
		#t.transform.rotation.w = qw
		##br.sendTransform(t)

		self.ekf_odom_msg.header.stamp = rospy.Time.now()
		self.ekf_odom_msg.header.frame_id = "odom"
		self.newOdom.child_frame_id = "base_link"
		self.ekf_odom_msg.pose.pose.position.x = x
		self.ekf_odom_msg.pose.pose.position.y = y
		self.ekf_odom_msg.pose.pose.position.z = z
		self.ekf_odom_msg.pose.pose.orientation.x = qx
		self.ekf_odom_msg.pose.pose.orientation.y = qy
		self.ekf_odom_msg.pose.pose.orientation.z = qz
		self.ekf_odom_msg.pose.pose.orientation.w = qw

		self.ekf_odom_msg.pose.covariance = data.pose.covariance

		self.newOdom.header.stamp = self.ekf_odom_msg.header.stamp
		self.newOdom.header.frame_id = self.ekf_odom_msg.header.frame_id = "odom"
		self.newOdom.pose = self.ekf_odom_msg.pose
		self.newOdom.twist.twist.linear.x = self.odomMsg.twist.twist.linear.x
		#self.newOdom.twist.twist.angular.z = self.odomMsg.twist.twist.angular.z
		self.newOdom.twist.twist.angular.z = self.imuMsg.angular_velocity.z
		self.ekf_odom_pub.publish(self.newOdom)
		return

	def main(self):
		rospy.spin()

if __name__ == '__main__':
	try:
		ekfNode = ekf_fusion('EKF_FUSION_NODE')
	except rospy.ROSInterruptException:
		pass
