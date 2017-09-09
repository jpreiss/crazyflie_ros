#!/usr/bin/env python

import rospy
import tf
from math import sin, cos, pi
from geometry_msgs.msg import Twist, Pose, PointStamped, TransformStamped
from std_srvs.srv import Empty
from crazyflie_driver.msg import FullyActuatedState, GenericLogData
from crazyflie_driver.srv import UpdateParams
from thrust_vis import ThrustVis

if __name__ == '__main__':

	rospy.init_node('crazyflie_demo_fully_actuated', anonymous=True)
	pub_setpoint = rospy.Publisher('crazyflie/cmd_fully_actuated', FullyActuatedState)
	pub_pose = rospy.Publisher('crazyflie/external_pose', Pose)

	rospy.loginfo("waiting for update_params service")
	rospy.wait_for_service('crazyflie/update_params')
	rospy.loginfo("found update_params service")
	update_params_srv = rospy.ServiceProxy('crazyflie/update_params', UpdateParams)

	rospy.loginfo("waiting for emergency service")
	rospy.wait_for_service('crazyflie/emergency')
	rospy.loginfo("found emergency service")
	emergency_srv = rospy.ServiceProxy('crazyflie/emergency', Empty)

	pose = Pose()
	pose.position.x = 0
	pose.position.y = 0
	pose.position.z = 0
	pose.orientation.x = 0
	pose.orientation.y = 0
	pose.orientation.z = 0
	pose.orientation.w = 1

	state = FullyActuatedState()

	state.pose.position.x = 0
	state.pose.position.y = 0
	state.pose.position.z = 0
	state.pose.orientation.x = 0
	state.pose.orientation.y = 0
	state.pose.orientation.z = 0
	state.pose.orientation.w = 1
	state.acc.x = 0
	state.acc.y = 0
	state.acc.z = 0

	state.twist.linear.x = 0
	state.twist.linear.y = 0
	state.twist.linear.z = 0
	state.twist.angular.x = 0
	state.twist.angular.y = 0
	state.twist.angular.z = 0

	RATE_HZ = 250
	r = rospy.Rate(RATE_HZ)

	t0 = rospy.get_rostime()

	vis = ThrustVis()
	thrusts_normalized = [0 for i in range(6)]

	def thrusts_callback(thrusts):
		global thrusts_normalized
		OMEGA2_MAX = 4.3865e6;
		thrusts_normalized = [w2 / OMEGA2_MAX for w2 in thrusts.values[0:6]]

	rospy.Subscriber('crazyflie/tilthexThrusts', GenericLogData, thrusts_callback)

	while not rospy.is_shutdown():
		x_frac = 2.000
		state.pose.position.x = x_frac
		pub_setpoint.publish(state)

		# using initial zero pose
		pub_pose.publish(pose)

		#vis.set_thrusts(thrusts_normalized)
		r.sleep()
