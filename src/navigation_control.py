#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import rospy

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String


goalPoints = [ 
    # from point1 to point2, to point3, to point4 and then back to point1
    # position[XYZ] and pose[quaternion]
    # In our map of lab, X-direction is from bottom to top and Y-direction is from right to left
    [(0.5152, -0.5065, 0.0), (0.0, 0.0, 0.9231, -0.3846)],  #pose of point1 index:0
    [(3.3856, 2.5524, 0.0), (0.0, 0.0, 0.9334, 0.3587)],    #pose of point2 index:1
    [(0.5313, 5.3832, 0.0), (0.0, 0.0, 0.9434, -0.3315)],   #pose of point3 index:2
    [(-2.3655, 2.4412, 0.0), (0.0, 0.0, -0.4172, 0.9088)],  #pose of point4 index:3
    [(0.5152, -0.5065, 0.0), (0.0, 0.0, 0.9334, 0.3587)],   #initial pose   index:4
    [(0.6053, 0.7533, 0.0), (0.0, 0.0, 0.4026, 0.9153)],  #pose of start point   index:5
]

class AutoNav:
    def __init__(self):
        self.moveBaseAction = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)


        wait_status = self.moveBaseAction.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Waiting for move_base action server...")
        if not wait_status:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        
        rospy.loginfo("Connected to move base server!")
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.moveBaseAction.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def set_initial_pose(self):
        '''To set the 2D pose estimate of initial moment'''
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = 0.6567
        init_pose.pose.pose.position.y = -0.4185
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = 0.9334
        init_pose.pose.pose.orientation.w = 0.3587
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] =  0.06853892326654787
        self.init_pose_pub.publish(init_pose)

    def set_goal(self, index):
        '''
        To produce and return the goal pose variable which contains position and orientation
        '''
        goal_pose = MoveBaseGoal()
        pose = goalPoints[index]
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    def move_goal(self, index, wait_time=60):
        '''To send the move command and wait for the result'''
        goal = self.set_goal(index)
        self.moveBaseAction.send_goal(goal)
        # wait_time = 60   #unit:seconds
        rospy.loginfo("Begin to navigate autonomous to the point"+str(index+1))
        finish_status = self.moveBaseAction.wait_for_result(rospy.Duration(wait_time))
        if not finish_status:
            self.moveBaseAction.cancel_goal()
            rospy.loginfo(str(wait_time)+" seconds time out without reaching the goal")
        else:
            if self.moveBaseAction.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Arrive the point "+str(index+1))

if __name__ == '__main__':

    rospy.init_node("navController_class")
    cmd = AutoNav()
    cmd.set_initial_pose()
    cmd.move_goal(1)
    cmd.move_goal(2)
    rospy.spin()