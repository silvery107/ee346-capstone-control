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

    # Initialize
    nav.set_initial_pose()
    follower_status = follower.initialize(timeout=40)

    # Start task sequences
    nav.move_goal(5)
    while follower_status is not FollowingStatus.SUCCEEDED:
        follower_status = follower.run()
        if follower_status is FollowingStatus.LOST:
            rospy.loginfo("Follower status: " + follower_status.name)
            break
        ctrl_rate.sleep()
    cv2.destroyAllWindows()
    nav.move_goal(2)
    nav.move_goal(3)
    nav.move_goal(1)
    nav.move_goal(0)
    rospy.spin()