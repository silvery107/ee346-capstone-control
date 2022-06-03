#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from navigation_control import AutoNav
from racetrack_control import Follower
import rospy
import cv2
from utils import FollowingStatus

if __name__ == "__main__":
    rospy.init_node("CMD_Integrated_System")
    ctrl_rate = rospy.Rate(25)
    # Instance core classes
    nav = AutoNav()
    follower = Follower()

    # Initial pose estimation
    nav.set_initial_pose()

    # Start task sequences
    # TODO add timers for each sub-task
    # Round 1
    nav.move_goal(5)
    follower.run(timeout=39)
    nav.move_goal(2)
    nav.move_goal(3)
    nav.move_goal(1)
    nav.move_goal(0)

    follower.refresh_aruco_search()
    # Round 2
    nav.move_goal(5)
    follower.run(timeout=39)
    nav.move_goal(2)
    nav.move_goal(3)
    nav.move_goal(1)
    nav.move_goal(0)

    rospy.spin()