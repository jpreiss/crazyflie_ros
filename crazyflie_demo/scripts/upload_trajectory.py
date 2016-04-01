#!/usr/bin/env python

import rospy
import argparse
import numpy as np
from crazyflie_driver.msg import QuadcopterTrajectoryPoint
from crazyflie_driver.srv import UploadTrajectory

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="input csv file")
    args = parser.parse_args()

    rospy.init_node('upload_trajectory', anonymous=True)

    service = "crazyflie/upload_trajectory"

    rospy.loginfo("wait for service")
    rospy.wait_for_service(service)
    rospy.loginfo("found upload_trajectory service")

    upload_trajectory = rospy.ServiceProxy(service, UploadTrajectory)

    matrix = np.loadtxt(args.csv_file, delimiter=',', skiprows=1)

    # t = UploadTrajectory()
    points = []
    for row in matrix:
        p = QuadcopterTrajectoryPoint()
        p.time_from_start = rospy.Duration.from_sec(row[0])
        p.position.x = row[1]
        p.position.y = row[2]
        p.position.z = row[3]
        p.velocity.x = row[4]
        p.velocity.y = row[5]
        p.velocity.z = row[6]
        p.yaw = row[7]
        points.append(p)

    upload_trajectory(points)
