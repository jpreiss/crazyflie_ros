#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
import numpy as np
from crazyflie_driver.msg import QuadcopterTrajectoryPoint
from crazyflie_driver.srv import UploadTrajectory

class Controller():
    def __init__(self, use_controller, joy_topic, csv_file):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        rospy.loginfo("wait for upload_trajectory service")
        rospy.wait_for_service("upload_trajectory")
        rospy.loginfo("found upload_trajectory service")
        self._upload_trajectory = rospy.ServiceProxy("upload_trajectory", UploadTrajectory)

        if use_controller:
            rospy.loginfo("waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo("found land service")
            self._land = rospy.ServiceProxy('land', Empty)

            rospy.loginfo("waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo("found takeoff service")
            self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        else:
            self._land = None
            self._takeoff = None

        self._csv_file = csv_file

        rospy.set_param("ctrlMel/massThrust", 135000.0)
        rospy.set_param("ctrlMel/kp", 0.2)
        rospy.set_param("ctrlMel/kd", 0.1)

        self._update_params(["ctrlMel/massThrust", "ctrlMel/kp", "ctrlMel/kd"])

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land != None:
                    self._land()
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
                if i == 2 and data.buttons[i] == 1 and self._takeoff != None:
                    self._takeoff()
                if i == 3 and data.buttons[i] == 1:
                    print("execute trajectory!")
                    x_offset = rospy.get_param("~x_offset", 0.0)
                    y_offset = rospy.get_param("~y_offset", 0.0)
                    matrix = np.loadtxt(self._csv_file, delimiter=',', skiprows=1)
                    points = []
                    for row in matrix:
                        p = QuadcopterTrajectoryPoint()
                        p.time_from_start = rospy.Duration.from_sec(row[0])
                        p.position.x = row[1] + x_offset
                        p.position.y = row[2] + y_offset
                        p.position.z = row[3]
                        p.velocity.x = row[4]
                        p.velocity.y = row[5]
                        p.velocity.z = row[6]
                        p.yaw = row[7]
                        points.append(p)
                    self._upload_trajectory(points)
                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))
                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)
                    self._update_params(["ring/headlightEnable"])
                    print(not value)

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    csv_file = rospy.get_param("~csv_file")
    controller = Controller(use_controller, joy_topic, csv_file)
    rospy.spin()
