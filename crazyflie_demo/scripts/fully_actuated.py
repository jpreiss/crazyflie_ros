#!/usr/bin/env python

import rospy
import tf
from math import sin, cos, pi
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty
from crazyflie_driver.msg import FullyActuatedState
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_fully_actuated', anonymous=True)
    p = rospy.Publisher('crazyflie/cmd_fully_actuated', FullyActuatedState)



    rospy.loginfo("waiting for update_params service")
    rospy.wait_for_service('crazyflie/update_params')
    rospy.loginfo("found update_params service")
    update_params_srv = rospy.ServiceProxy('update_params', UpdateParams)

    rospy.loginfo("waiting for emergency service")
    rospy.wait_for_service('crazyflie/emergency')
    rospy.loginfo("found emergency service")
    emergency_srv = rospy.ServiceProxy('emergency', Empty)

    # rospy.loginfo("waiting for land service")
    # rospy.wait_for_service('crazyflie/land')
    # rospy.loginfo("found land service")
    # land_srv = rospy.ServiceProxy('land', Empty)

    # rospy.loginfo("waiting for takeoff service")
    # rospy.wait_for_service('crazyflie/takeoff')
    # rospy.loginfo("found takeoff service")
    # takeoff_srv = rospy.ServiceProxy('takeoff', Empty)



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

    RATE = 50.0 #Hz
    DT = 1.0 / RATE
    r = rospy.Rate(RATE)

    #raw_input("press enter to take off")
    takeoff_height = 0.75
    takeoff_sec = 10
    #takeoff_srv()


    #raw_input("press enter to oscillate in place")
    THETA_RAD = 0.5
    PERIOD_SEC = 2 * pi
    RAMP_SEC = 2.0
    HOLD_SEC = 3.0
    RATE_HZ = 50
    T_RAMPDOWN = RAMP_SEC + HOLD_SEC
    T_END = T_RAMPDOWN + RAMP_SEC
    print "T_END ==", T_END

    time_scale = (2 * pi) / PERIOD_SEC
    r = rospy.Rate(RATE_HZ)

    t0 = rospy.get_rostime()

    while not rospy.is_shutdown():
        t = (rospy.get_rostime() - t0).to_sec()

        if t < RAMP_SEC:
            theta_scale = (t / RAMP_SEC) * THETA_RAD
        elif t < T_RAMPDOWN:
            theta_scale = THETA_RAD
        elif t > T_END:
            theta_scale = 0
            break
        elif t > T_RAMPDOWN:
            theta_scale = ((RAMP_SEC - (t - T_RAMPDOWN)) / RAMP_SEC) * THETA_RAD
        else:
            assert(False)

        th = theta_scale * sin(time_scale * t)
        dth = theta_scale * time_scale * cos(time_scale * t)

        q = tf.transformations.quaternion_from_euler(0, 0, th, 'rzyx')
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        state.twist.angular.x = dth

        p.publish(state)
        r.sleep()


    #raw_input("press enter to land")
    #land_srv()
