#! /usr/bin/env python

import roslib
roslib.load_manifest('crazyflie_demo')
import rospy
import actionlib
import numpy as np
import argparse

from crazyflie_controller.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal, QuadcopterTrajectoryPoint

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="input csv file")
    args = parser.parse_args()

    rospy.init_node('execute_trajectory_client')
    client = actionlib.SimpleActionClient('/crazyflie/execute_trajectory', ExecuteTrajectoryAction)
    client.wait_for_server()

    matrix = np.loadtxt(args.csv_file, delimiter=',', skiprows=1)

    goal = ExecuteTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    for row in matrix:
        pt = QuadcopterTrajectoryPoint()
        pt.position.x = row[1]
        pt.position.y = row[2]
        pt.position.z = row[3]
        pt.velocity.x = row[4]
        pt.velocity.y = row[5]
        pt.velocity.z = row[6]
        pt.acceleration.x = row[7]
        pt.acceleration.y = row[8]
        pt.acceleration.z = row[9]
        pt.yaw = row[10]
        pt.time_from_start = rospy.Duration.from_sec(row[0])
        goal.trajectory.points.append(pt)
    client.send_goal(goal)
    # client.wait_for_result(rospy.Duration.from_sec(5.0))
