#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from controllers import PIDController, NonholomonicController
from moving_window_filter import MovingWindowFilter
import warnings
warnings.simplefilter('ignore', np.RankWarning)
from utils import check_coord, match_corner, get_lane_theta, FollowingStatus
from argparse import ArgumentParser

####################################################
# CONSTANTS
DTYPE = np.float32
STOP_DISTANCE = 0.055
CROSS_DISTANCE = 0.4 # 0.28
STOP_TIME = 1
kernel = cv2.getGaussianKernel(5, 5)
rot_90 = np.array([0,1,-1,0], dtype=DTYPE).reshape((2,2))

# Camera params and Homography
# resolution (320, 240)
IMG_H = 240
IMG_W = 320
distCoeffs = np.array([0.135529, -0.197880, 0.009788, -0.003316, 0.000000], dtype=DTYPE)
cameraMatrix = np.array([273.9783, 0.0, 151.81899, 0.0, 273.03021, 127.88242, 0.0, 0.0, 1.0],
                dtype=DTYPE).reshape((3,3))
top_x = 105 #88 # 47
top_y = 12 # 38
bottom_x = 190
bottom_y = 90 #120 
# selecting 4 points from the original image
pts_src = np.array([[160 - top_x, 180 - top_y], 
                    [160 + top_x, 180 - top_y], 
                    [160 + bottom_x, 120 + bottom_y], 
                    [160 - bottom_x, 120 + bottom_y]],
                    dtype=DTYPE)

# selecting 4 points from image that will be transformed
LANE_LEFT = 53.3333
LANE_RIGHT = 266.6667
pts_dst = np.array([[LANE_LEFT, 0], [LANE_RIGHT, 0], [LANE_RIGHT, IMG_H], [LANE_LEFT, IMG_H]], dtype=DTYPE)
# finding homography matrix
homography, status = cv2.findHomography(pts_src, pts_dst)
homography /= homography[2,2]
print(homography)

# ArUco stuff
ARUCO_TAG = cv2.aruco.DICT_6X6_50
aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_TAG)
aruco_parameters = cv2.aruco.DetectorParameters_create()
####################################################

class Follower:

    def __init__(self, disable_motor=False, test_aruco=False):
        # Components
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # 10
        self.twist = Twist()
        self.image = None

        # Stop State
        self.stop = False
        self.stop_once = False
        self.timer = None
        self.positions = np.zeros(3, dtype=DTYPE)
        self.stop_pos = None

        # Controllers and filters
        self.controller = NonholomonicController(0.01, 1.0, 0.5, max_v=0.22, max_w=2.84)
        self.filter_x = MovingWindowFilter(1, dim=1)
        self.filter_y = MovingWindowFilter(1, dim=1)
        self.filter_t = MovingWindowFilter(2, dim=1)

        # Turning State
        self.turn_left = False
        self.turn_right = False
        self.mis_left = False
        self.mis_right = False

        # Crossing State
        self.cross_flag = False
        self.cross_once = False
        self.cross_counter = 1
        self.cross_pos = None
        self.corner_templates = [cv2.imread("/home/lab/catkin_ws/src/ee346-capstone-control/src/templates/corner_template_squ.png", 0), 
                                cv2.imread("/home/lab/catkin_ws/src/ee346-capstone-control/src/templates/corner_template_rec.png", 0),
                                cv2.imread("/home/lab/catkin_ws/src/ee346-capstone-control/src/templates/corner_template_sharp.png", 0)]
        # Start & Exit State
        self.start = False
        self.start_once = False
        self.exit = False
        self.exit_once = False
        self.exit_counter_num = 2

        # Testing Flag
        self.disable_motor = disable_motor
        self.test_aruco = test_aruco
        self.stop_twist = Twist()
        rospy.on_shutdown(self.shutdown_hook)

        # self.cmd_vel_pub.publish(self.stop_twist)
        # rospy.sleep(1)

    def shutdown_hook(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(self.stop_twist)
        self.stop_once = True
        self.exit_once = True
        # cv2.destroyAllWindows()
        rospy.sleep(1)
        
    def print_state(self):
        print("Turn Left:", self.turn_left, "Turn Right:", self.turn_right)
        print("Miss Left:", self.mis_left, "Miss Right:", self.mis_right)
        print("Cross Flag:", self.cross_flag, "Cross Counter:", self.cross_counter)
        # print("Stop Flag:", self.stop_flag)

    def start_seq(self):
        self.start = False
        self.twist.linear.x = 0.18
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2.5)
        self.twist.linear.x = 0.0
        self.twist.angular.z = -1.5
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1)
        self.start_once = True

    def exit_seq(self):
        self.exit = False
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2.3)
        self.twist.linear.x = 0
        self.twist.angular.z = -1.5
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1)
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(4)
        self.exit_once = True

    def stop_seq(self):
        self.stop = False
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)
        self.cmd_vel_pub.publish(self.stop_twist)
        rospy.sleep(STOP_TIME)
        self.stop_once = True
        self.stop_pos = self.positions.copy()

    def odom_callback(self, msg):
        self.positions[0] = msg.pose.pose.position.x
        self.positions[1] = msg.pose.pose.position.y
        self.positions[2] = msg.pose.pose.position.z

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def run(self):
        if self.timer is None:
            rospy.loginfo("Start lane following timer!")
            self.timer = rospy.get_time()

        if self.image is None:
            rospy.loginfo("Waitting image...")
            rospy.sleep(1)
            return

        self.turn_left = False
        self.turn_right = False
        self.mis_left = False
        self.mis_right = False

        #### *ArUco Detection #####
        # if self.cross_counter == 3 or self.test_aruco:
        corners, ids, _ = cv2.aruco.detectMarkers(self.image, aruco_dictionary, parameters=aruco_parameters)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(self.image, corners, ids)
            _, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs)
            # print(tvecs.squeeze()[-1])
            if tvecs.squeeze()[-1] < STOP_DISTANCE*10 and not self.stop and not self.stop_once:
                self.stop = True
                rospy.loginfo("[Stop] Flag Triggered")
                self.timer = rospy.get_time()

        #### *Perspective Transform #####
        img_bird_view = cv2.warpPerspective(self.image, homography, (IMG_W, IMG_H))

        span = np.linspace(0, 72, IMG_H)
        for row in range(5, IMG_H):
            img_bird_view[row, 0:int(span[row])] = 255
            img_bird_view[row, IMG_W - int(span[row]):] = 255

        #### *Image Processing #####
        img_hsv = cv2.cvtColor(img_bird_view, cv2.COLOR_BGR2HSV)
        h, w, d = self.image.shape # (240, 320, 3)

        lower_black = np.array([0, 0, 0], dtype=DTYPE)
        upper_black = np.array([180, 255, 90], dtype=DTYPE)

        mask1 = cv2.inRange(img_hsv[:,:w/2], lower_black, upper_black)
        mask2 = cv2.inRange(img_hsv[:,w/2:], lower_black, upper_black)

        # !Blind top 1/3
        # search_top = h*1/3
        # mask1[0:search_top, 0:w] = 0
        # mask2[0:search_top, 0:w] = 0

        mask_add = np.zeros((h,w), dtype=DTYPE)
        mask_add[:, :w/2] = mask1
        mask_add[:, w/2:] = mask2
        # remove white points
        mask_add = cv2.morphologyEx(mask_add, cv2.MORPH_OPEN, kernel, iterations=3)

        #### *Calc Lane Orientation #####
        theta1 = get_lane_theta(mask1, kernel)
        theta2 = get_lane_theta(mask2, kernel)
        theta = 0.0

        if abs(theta1) > abs(theta2) and abs(theta1) > 0.3:
            theta = theta2
            self.turn_left = True
        elif abs(theta2) > abs(theta1) and abs(theta2) > 0.3:
            theta = theta1
            self.turn_right = True
        else:
            theta = (theta1 + theta2) / 2.0
        # print("theta1:%.3f, theta2: %.3f"%(theta1, theta2))
        # TODO decide which to use
        theta = self.filter_t.calculate_average((theta1 + theta2) / 2)
        # print("theta: %.3f"%theta)

        #### *Calc Lane Moment #####
        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)
        # TODO are they good init vals?
        cx1 = LANE_LEFT
        cx2 = LANE_RIGHT
        cy1 = IMG_H/2
        cy2 = IMG_H/2

        # TODO tuning missing
        MOM_THR = 400000 
        if M1['m00'] > MOM_THR:
            cx1 = M1['m10']/M1['m00']
            cy1 = M1['m01']/M1['m00']
            # print("M1:", M1['m00'])
        else:
            self.mis_left = True
            # print("Miss Left")

        if M2['m00'] > MOM_THR:
            cx2 = M2['m10']/M2['m00'] + w/2
            cy2 = M2['m01']/M2['m00']
            # print("M2:", M2['m00'])
        else:
            self.mis_right = True
            # print("Miss Right")

        #### *Desition Logic #####
        # TODO tuning turn center
        # self.print_state()
        if self.turn_left:
            cx1 = cx2 - 150
            cy1 = cy2
        elif self.turn_right:
            cx2 = cx1 + 150
            cy2 = cy1
        else:
            if match_corner(mask2, self.corner_templates):
                rospy.loginfo("Corner Matched!")
                if self.cross_counter == 1 and not self.start and not self.start_once:
                    self.start = True
                    rospy.loginfo("[Start] Flag Triggered")
                elif not self.cross_flag and not self.cross_once and self.start_once:
                    self.cross_counter += 1
                    rospy.loginfo("[Cross] Counts: %d"%self.cross_counter)
                    self.cross_flag = True
                    rospy.loginfo("[Cross] Flag Triggered")
                    if self.cross_counter == self.exit_counter_num and self.start_once:
                        self.exit = True
                        rospy.loginfo("[Exit] Flag Triggered")

            if self.mis_right and not self.mis_left:
                cx2 = IMG_W - cx1 +10
                cy2 = cy1
            elif self.mis_left and not self.mis_right:
                cx1 = IMG_W - cx2 -10
                cy1 = cy2

        fpt_x = (cx1 + cx2)/2
        fpt_y = (cy1 + cy2)/2

        #### *Stop Logic #####        
        # if self.stop_once:
        #     distance = np.linalg.norm((self.positions-self.stop_pos))
        #     # print(distance)
        #     if distance > STOP_DISTANCE:
        #         self.stop_once = False
        #         self.cross_counter = 1
        #         rospy.loginfo("[Cross] Refresh Counts: %d"%self.cross_counter)
        #         rospy.loginfo("[Stop] Refresh State")

        #### *Crossing Logic #####
        if self.cross_flag:
            self.cross_pos = self.positions.copy()
            self.cross_once = True
            self.cross_flag = False
        
        if self.cross_once:
            distance = np.linalg.norm((self.positions-self.cross_pos))
            if distance > CROSS_DISTANCE:
                self.cross_once = False
                rospy.loginfo("[Cross] Refresh State")

        #### *Execute Controller #####
        dx = self.filter_x.calculate_average(fpt_y/10.0)
        dy = self.filter_y.calculate_average(w/2 - fpt_x)

        v, omega = self.controller.apply(dx, dy, theta)
        self.twist.linear.x = 0.22 if not (self.turn_left or self.turn_right) else 0.18
        self.twist.angular.z = omega

        #### *Command Publish #####
        if not self.disable_motor:
            if self.start:
                self.start_seq()
            elif self.exit:
                self.exit_seq()
            elif self.stop:
                self.stop_seq()

            self.cmd_vel_pub.publish(self.twist)
        else:
            self.cmd_vel_pub.publish(self.stop_twist)

        #### *Image Display #####
        cv2.namedWindow("masks")
        cv2.moveWindow("masks", 50+2*IMG_W, 200)
        cv2.namedWindow("image")
        cv2.moveWindow("image", 50, 200)
        # cv2.imshow("BEV", img_bird_view)
        # cv2.imshow("HSV", img_hsv)
        cv2.imshow("masks", mask_add)

        cv2.circle(img_bird_view, (int(cx1), int(cy1)), 10, (0, 255, 255), -1)
        cv2.circle(img_bird_view, (int(cx2), int(cy2)), 10, (255, 255, 255), -1)
        cv2.circle(img_bird_view, (int(fpt_x), int(fpt_y)), 10, (128, 128, 128), -1)

        cv2.line(img_bird_view, (w/2-dy*5, h), (w/2-dy*5, h-dx*5), (0, 0, 255), 2)
        cv2.line(img_bird_view, (w/2, h-2), (w/2-dy*5, h-2), (0, 0, 255), 2)

        img_pair = np.concatenate((self.image, img_bird_view), axis=1)
        cv2.imshow("image", img_pair)
        cv2.waitKey(1)

        #### *Following Status #####
        if rospy.get_time() - self.timer>self.timeout:
            return FollowingStatus.LOST
        elif self.exit_once:
            return FollowingStatus.SUCCEEDED
        else:
            return FollowingStatus.ACTIVE

    def initialize(self, timeout=40):
        self.timeout = timeout
        return FollowingStatus.PENDING

if __name__ == '__main__':
    # parser = ArgumentParser(prog="Racetrack Control")
    # parser.add_argument('--disable-motor', action='store_true')
    # parser.add_argument('--test-aruco', action='store_true')
    # args = parser.parse_args()

    rospy.init_node('lane_follower')
    ctrl_rate = rospy.Rate(25)
    follower = Follower()
    # follower = Follower(args.disable_motor, args.test_aruco)
    while not follower.stop_once:
        follower.run()
        ctrl_rate.sleep()
    # rospy.spin()
