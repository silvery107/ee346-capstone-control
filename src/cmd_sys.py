#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from navigation_control import AutoNav
from racetrack_control import Follower
import rospy
import cv2

if __name__ == "__main__":
    rospy.init_node("CMD_Integrated_System")

    ctrl_rate = rospy.Rate(25)
    nav = AutoNav()
    follower = Follower()
    
    nav.set_initial_pose()
    nav.move_goal(5)
    while not follower.exit_once:
        follower.run()
        ctrl_rate.sleep()
    cv2.destroyAllWindows()
    nav.move_goal(2)
    nav.move_goal(3)
    nav.move_goal(1)
    nav.move_goal(0)
    rospy.spin()