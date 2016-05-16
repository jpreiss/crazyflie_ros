#!/usr/bin/env python

import rospy
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('enable_pos_ctrl', anonymous=True)
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)
    rospy.set_param("flightmode/posCtrl", 1)
    update_params(["flightmode/posCtrl"])
