#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from navigation_control import AutoNav
from racetrack_control import Follower
import rospy

if __name__ == "__main__":
    rospy.init_node("CMD_Integrated_System")
    ctrl_rate = rospy.Rate(25)
    # Instance core classes
    nav = AutoNav()
    follower = Follower()

    # Initial pose estimation
    nav.set_initial_pose()

    # Start task sequences
    # Round 1
    nav.move_goal(5) #start point of racetrack
    follower.run({"timeout":15})
    nav.move_goal(1) #point2
    nav.move_goal(6) #back point of racetrack
    follower.run({"timeout":24, "start_once":True})
    nav.move_goal(2) #point3
    nav.move_goal(3) #point4
    # nav.move_goal(7) #search point for aruco
    nav.move_goal(7) #search for a mid-connection point
    nav.move_goal(0) #point1


    # Round 2
    follower.refresh_aruco_search()
    nav.move_goal(5) #start point of racetrack
    follower.run({"timeout":15})
    nav.move_goal(1) #point2
    nav.move_goal(6) #back point of racetrack
    follower.run({"timeout":22, "start_once":True})
    nav.move_goal(2) #point3
    nav.move_goal(3) #point4
    # nav.move_goal(7) #search point for aruco
    nav.move_goal(7) #search for a mid-connection point
    nav.move_goal(0) #point1


    # Round 3
    follower.refresh_aruco_search()
    nav.move_goal(5) #start point of racetrack
    follower.run({"timeout":39})
    nav.move_goal(3) 
    follower.stop_seq()

    # TEST
    # nav.move_goal(1)
    # nav.move_goal(0) #point1
    # nav.move_goal(1)

    # follower.rotate_seq(time=0.7, speed=-2)
    # nav.move_goal(5) #start point of racetrack
    # nav.move_goal(0) #point1
    # follower.rotate_seq(time=0.7, speed=-2)
    # nav.move_goal(5) #start point of racetrack
    # nav.move_goal(0) #point1

    rospy.spin()