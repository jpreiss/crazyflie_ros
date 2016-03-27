#! /usr/bin/env python

import roslib
roslib.load_manifest('crazyflie_demo')
import rospy
import actionlib

from crazyflie_controller.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal, QuadcopterTrajectoryPoint

if __name__ == '__main__':
    rospy.init_node('execute_trajectory_client')
    client = actionlib.SimpleActionClient('/crazyflie/execute_trajectory', ExecuteTrajectoryAction)
    client.wait_for_server()

    goal = ExecuteTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    pt = QuadcopterTrajectoryPoint()
    pt.position.z = 0.5
    pt.position.x = 0.5
    goal.trajectory.points.append(pt)
    client.send_goal(goal)
    # client.wait_for_result(rospy.Duration.from_sec(5.0))
