#!/usr/bin/env python

import rospy
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('enable_pos_ctrl', anonymous=True)
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)
    rospy.set_param("flightmode/posCtrl", 1)
    rospy.set_param("ctrlMel/kp_xy", 0.3)
    rospy.set_param("ctrlMel/kd_xy", 0.3)
    rospy.set_param("ctrlMel/ki_xy", 0.05)
    rospy.set_param("ctrlMel/i_range_xy", 2.0)
    rospy.set_param("ctrlMel/kR_xy", 9000)
    rospy.set_param("ctrlMel/kw_xy", 4000)

    rospy.set_param("ctrlMel/kR_z", 10000)
    rospy.set_param("ctrlMel/kw_z", 3000)
    rospy.set_param("ctrlMel/ki_m_z", 100)
    rospy.set_param("ctrlMel/i_range_m_z", 100)

    rospy.set_param("ctrlMel/kp_z", 0.9)
    rospy.set_param("ctrlMel/kd_z", 0.5)
    rospy.set_param("ctrlMel/ki_z", 0.2)
    rospy.set_param("ctrlMel/i_range_z", 1.5)

    rospy.set_param("ctrlMel/mass", 0.032)
    rospy.set_param("ctrlMel/massThrust", 115000)

    rospy.set_param("ring/effect", 6)
    rospy.set_param("ring/headlightEnable", 0)

    update_params(["flightmode/posCtrl", "ctrlMel/kp_xy", "ctrlMel/kd_xy", "ctrlMel/kR_xy", "ctrlMel/kw_xy", "ctrlMel/ki_xy", "ctrlMel/i_range_xy", "ctrlMel/kp_z", "ctrlMel/kd_z", "ctrlMel/ki_z", "ctrlMel/i_range_z", "ctrlMel/kR_z", "ctrlMel/kw_z", "ctrlMel/ki_m_z", "ctrlMel/i_range_m_z", "ctrlMel/mass", "ctrlMel/massThrust", "ring/effect", "ring/headlightEnable"])
