#!/usr/bin/env python

import rospy
import tf
import copy
from math import sin, cos, pi
from geometry_msgs.msg import Twist, Pose, PointStamped, TransformStamped
from std_srvs.srv import Empty
from crazyflie_driver.msg import FullyActuatedState, GenericLogData
from crazyflie_driver.srv import UpdateParams
from thrust_vis import ThrustVis
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

if __name__ == '__main__':

	rospy.init_node('crazyflie_demo_fully_actuated', anonymous=True)
	pub_setpoint = rospy.Publisher('crazyflie/cmd_fully_actuated', FullyActuatedState)
	pub_roll = rospy.Publisher('crazyflie/roll', Float32)
	#pub_vicon = rospy.Publisher('crazyflie/external_position', PointStamped)
	pub_pose = rospy.Publisher('crazyflie/external_pose', Pose)

	rospy.loginfo("waiting for update_params service")
	rospy.wait_for_service('crazyflie/update_params')
	rospy.loginfo("found update_params service")
	update_params_srv = rospy.ServiceProxy('crazyflie/update_params', UpdateParams)

	rospy.loginfo("waiting for emergency service")
	rospy.wait_for_service('crazyflie/emergency')
	rospy.loginfo("found emergency service")
	emergency_srv = rospy.ServiceProxy('crazyflie/emergency', Empty)

	# rospy.loginfo("waiting for land service")
	# rospy.wait_for_service('crazyflie/land')
	# rospy.loginfo("found land service")
	# land_srv = rospy.ServiceProxy('land', Empty)

	# rospy.loginfo("waiting for takeoff service")
	# rospy.wait_for_service('crazyflie/takeoff')
	# rospy.loginfo("found takeoff service")
	# takeoff_srv = rospy.ServiceProxy('takeoff', Empty)

	THRUST_SCALE = 1.25 # full battery: 1.25; empty battery: 1.50
	rospy.set_param("crazyflie/tilthex_dynamics/k_thrust", 1.0/THRUST_SCALE * 1.6e-6)
	rospy.set_param("crazyflie/tilthex_dynamics/k_drag", 1.0/THRUST_SCALE * 8.0e-8)
	rospy.set_param("crazyflie/tilthex_dynamics/mass", 1.126)
	rospy.set_param("crazyflie/tilthex_pid/att_kp", 60) # 80
	rospy.set_param("crazyflie/tilthex_pid/att_kd", 15) # 20
	rospy.set_param("crazyflie/tilthex_pid/pos_kp", 8)
	rospy.set_param("crazyflie/tilthex_pid/pos_kd", 4)
	update_params_srv([
		"tilthex_dynamics/k_thrust",
		"tilthex_dynamics/k_drag",
		"tilthex_dynamics/mass",
		"tilthex_pid/att_kp",
		"tilthex_pid/att_kd",
		"tilthex_pid/pos_kp",
		"tilthex_pid/pos_kd"])

	#vicon_pos = PointStamped()
	#vicon_pos.point.x = 0
	#vicon_pos.point.y = 0
	#vicon_pos.point.z = 0

	pose = Pose()
	pose.position.x = 0
	pose.position.y = 0
	pose.position.z = 0
	pose.orientation.x = 0
	pose.orientation.y = 0
	pose.orientation.z = 0
	pose.orientation.w = 1

	setpoint = FullyActuatedState()

	setpoint.pose.position.x = 0
	setpoint.pose.position.y = 0
	setpoint.pose.position.z = 0
	setpoint.pose.orientation.x = 0
	setpoint.pose.orientation.y = 0
	setpoint.pose.orientation.z = 0
	setpoint.pose.orientation.w = 1
	setpoint.acc.x = 0
	setpoint.acc.y = 0
	setpoint.acc.z = 0

	setpoint.twist.linear.x = 0
	setpoint.twist.linear.y = 0
	setpoint.twist.linear.z = 0
	setpoint.twist.angular.x = 0
	setpoint.twist.angular.y = 0
	setpoint.twist.angular.z = 0

	RATE = 50.0 #Hz
	DT = 1.0 / RATE
	r = rospy.Rate(RATE)

	#raw_input("press enter to take off")
	takeoff_height = 0.75
	takeoff_sec = 10
	#takeoff_srv()


	#raw_input("press enter to oscillate in place")
	THETA_RAD = 0
	PERIOD_SEC = 1
	RAMP_SEC = 2.0
	HOLD_SEC = 3.0
	RATE_HZ = 250
	T_RAMPDOWN = RAMP_SEC + HOLD_SEC
	T_END = T_RAMPDOWN + RAMP_SEC
	#print "T_END ==", T_END

	time_scale = (2 * pi) / PERIOD_SEC
	r = rospy.Rate(RATE_HZ)

	t0 = rospy.get_rostime()

	# vis = ThrustVis()
	thrusts_normalized = [0 for i in range(6)]

	vicon_init = False
	hold_pos = None

	logdata = open("logdata.csv", 'w')
	logdata.write("time,goaly,y\n")

	def xform_callback(xform):
		global vicon_init
		global hold_pos
		global pose
		pose.position.x = xform.transform.translation.x
		pose.position.y = xform.transform.translation.y
		pose.position.z = xform.transform.translation.z
		pose.orientation = xform.transform.rotation
		pub_pose.publish(pose)
		# r, p, y = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
		# rollMsg = Float32()
		# rollMsg.data = r
		# pub_roll.publish(rollMsg)
		# print "vicon_init =", vicon_init
		if not vicon_init:
			hold_pos = copy.deepcopy(pose.position)
			vicon_init = True
		
	rospy.Subscriber('vicon/tilthex/tilthex', TransformStamped, xform_callback)

	def thrusts_callback(thrusts):
		global thrusts_normalized
		OMEGA2_MAX = 4.3865e6;
		thrusts_normalized = [w2 / OMEGA2_MAX for w2 in thrusts.values[0:6]]

	def pwms_callback(pwms):
		global thrusts_normalized
		global acc
		def conv(pwm):
			omega2 = (pwm/0.4 - 3683) ** 2
			return omega2 / OMEGA2_MAX
		thrusts_normalized = [conv(p) for p in pwms.values[0:6]]

	def quat_callback(quat):
		#print "ext quat (xyzw): " + str(quat.values)
		pass

	def ekf_pos_callback(pos):
		global state
		logdata.write("{},{},{}\n".format(rospy.get_rostime().to_sec(), state.pose.position.y, pos.values[1]))
		#print "ekf pos: ", pos.values[0:3], ", usec: ", pos.values[3]
		pass

	def ekf_rpy_callback(rpy):
		# print ", ".join(str(x) for x in rpy.values[0:2])
		pass

	roll = 0
	running = False

	def joy_changed_callback(joy):
		# print(joy)
		global roll
		global running
		if joy.buttons[2] == 1: #blue
			roll = 0.1
		if joy.buttons[0] == 1: # green
			roll = 0.0
		if joy.buttons[7] == 1: # start
			running = True
		if joy.buttons[1] == 1: # red
			running = False
		# print("switched roll to {}".format(roll))

	rospy.Subscriber('crazyflie/tilthexThrusts', GenericLogData, thrusts_callback)
	rospy.Subscriber('crazyflie/ext_quat', GenericLogData, quat_callback)
	rospy.Subscriber('crazyflie/ekf_pos', GenericLogData, ekf_pos_callback)
	rospy.Subscriber('crazyflie/ekf_rpy', GenericLogData, ekf_rpy_callback)
	rospy.Subscriber('crazyflie/joy', Joy, joy_changed_callback)

	while not rospy.is_shutdown():
		t = (rospy.get_rostime() - t0).to_sec()

		if t < RAMP_SEC:
			theta_scale = (t / RAMP_SEC) * THETA_RAD
		elif t < T_RAMPDOWN:
			theta_scale = THETA_RAD
		elif t > T_END:
			#theta_scale = 0
			theta_scale = THETA_RAD
			#break
		elif t > T_RAMPDOWN:
			theta_scale = ((RAMP_SEC - (t - T_RAMPDOWN)) / RAMP_SEC) * THETA_RAD
		else:
			assert(False)

		th = theta_scale * sin(time_scale * t)
		dth = theta_scale * time_scale * cos(time_scale * t)

		x_scale = 0.2
		x =   x_scale *					sin(time_scale * t)
		dx =  x_scale *	  time_scale *  cos(time_scale * t)
		ddx = x_scale * time_scale ** 2 * -sin(time_scale * t)

		if vicon_init:
			# rollMsg = Float32()
			# rollMsg.data = roll
			# pub_roll.publish(rollMsg)
			q = tf.transformations.quaternion_from_euler(0, 0, 0) #, 'rzyx')
			setpoint.pose.orientation.x = q[0]
			setpoint.pose.orientation.y = q[1]
			setpoint.pose.orientation.z = q[2]
			setpoint.pose.orientation.w = q[3]
			setpoint.twist.angular.x = 0

			setpoint.pose.position.x = hold_pos.x
			setpoint.pose.position.y = hold_pos.y + roll
			# print(setpoint.pose.position.y)
			setpoint.pose.position.z = hold_pos.z
			setpoint.twist.linear.x = 0
			setpoint.twist.linear.y = 0
			setpoint.twist.linear.z = 0
			setpoint.acc.x = 0
			setpoint.acc.y = 0
			setpoint.acc.z = 0

			if running:
				pub_setpoint.publish(setpoint)


		#q = tf.transformations.quaternion_from_euler(th, 0, 0, 'rzyx')
		# fake pose - now using real pose
		#pose.position.x = 0
		#pose.position.y = 0
		#pose.position.z = 0
		#pose.orientation.x = q[0]
		#pose.orientation.y = q[1]
		#pose.orientation.z = q[2]
		#pose.orientation.w = q[3]
		#print pose.orientation
		#vicon_pos.point.x = 0.3 * x;
		#print "x =", vicon_pos.point.x

		#pub_vicon.publish(vicon_pos)
		#pub_pose.publish(pose)

		#vis.set_thrusts(thrusts_normalized)
		r.sleep()


	#raw_input("press enter to land")
	#land_srv()
