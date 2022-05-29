import cv2
import numpy as np
DTYPE = np.float32
rot_90 = np.array([0,1,-1,0], dtype=DTYPE).reshape((2,2))

def check_coord(coord):
    if coord[0]<0 and coord[1]<0:
        return -coord
    elif coord[0]<0 and coord[1]>0:
        return -coord
    else:
        return coord

def get_lane_theta(mask, kernel):
    theta = 0.0
    mask_eroded = cv2.erode(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask_eroded, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)[-2:]
    if contours:
        line_param = cv2.fitLine(contours[0], distType=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01)
        new_coord = rot_90.dot(np.array([line_param[0], line_param[1]],dtype=DTYPE).reshape((2,1)))
        new_coord = check_coord(new_coord) + 1e-5
        theta = np.arctan2(new_coord[1], new_coord[0])
        # print(np.abs(theta)-np.pi/2)
        if np.isclose(np.abs(theta), np.pi/2, 1e-4):
            theta = 0.0

    return theta

def match_corner(img, templates):

    for template in templates:
        match_res = cv2.matchTemplate(img, template, cv2.TM_SQDIFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(match_res)
        # print(min_val)
        if min_val < 0.55:
            # print("[Cross] Corner Matched")
            return True
        else:
            return False