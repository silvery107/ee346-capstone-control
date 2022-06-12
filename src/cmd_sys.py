#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from navigation_control import AutoNav
from racetrack_control import Follower
import rospy

if __name__ == "__main__":
    rospy.init_node("CMD_Integrated_System")
    # Instance core classes
    navigator = AutoNav()
    follower = Follower()

    # Initial pose estimation
    navigator.set_initial_pose()

    # Start task sequences
    # Round 1
    navigator.move_goal(5, timeout=10) #start point of racetrack
    follower.run({"timeout":15})
    navigator.move_goal(1) #point2
    navigator.move_goal(6) #back point of racetrack
    follower.run({"timeout":22, "start_once":True})
    navigator.move_goal(2) #point3
    navigator.move_goal(3) #point4
    # nav.move_goal(7) #search point for aruco
    navigator.move_goal(7) #search for a mid-connection point
    navigator.move_goal(0) #point1


    # Round 2
    follower.refresh_aruco_search()
    navigator.move_goal(5) #start point of racetrack
    follower.run({"timeout":15})
    navigator.move_goal(1) #point2
    navigator.move_goal(6) #back point of racetrack
    follower.run({"timeout":22, "start_once":True})
    navigator.move_goal(2) #point3
    navigator.move_goal(3) #point4
    # nav.move_goal(7) #search point for aruco
    navigator.move_goal(7) #search for a mid-connection point
    navigator.move_goal(0) #point1


    # Round 3
    follower.refresh_aruco_search()
    navigator.move_goal(5) #start point of racetrack
    follower.run({"timeout":39})
    navigator.move_goal(3) 
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