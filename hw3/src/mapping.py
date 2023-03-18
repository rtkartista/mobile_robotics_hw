#! /usr/bin/env python3
# The robot should ignore obstacles to the side.
# Only stops for stuff directly in front of it.

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from tf.transformations import euler_from_quaternion
import pdb
import grid
import time

class mapping:
    def __init__(self):
        # Initialize the node, and call it "mapping".
        rospy.init_node('mapping')

        # Set up a subscriber.  The default topic for LaserScan messages is base_scan.
        self.subscriber = rospy.Subscriber('base_scan', LaserScan, self.sub_laser, queue_size=1)
        
        # Set up a subscriber to get robot's orientation and position by the encoder data
        # Assuming we get true ground state
        # self.odom_sub = rospy.Subscriber('odom', Odometry, self.pose, queue_size=1)
        # /odometry/filtered
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.pose, queue_size=1)
        # get parameters
        self.max_linear = rospy.get_param('~v_max')
        self.sample_rate = rospy.get_param('~sample_rate')
        self.map_x = rospy.get_param('~map_x')
        self.map_y = rospy.get_param('~map_y')
        self.res = rospy.get_param('~res')
        
        # laser readings
        self.laser = None
        self.pose_data = None

        # Grid cells probability values [0, 1] [free, occupied]
        self.init_p = 0.5
        self.min_p = 0.1
        self.max_p = 0.99

        # grid object with mapping and probability update methods
        self.grid = grid.grid(x = self.map_y, y = self.map_x, resolution = self.res,
                          prior= self.init_p)

    # start spinning the simple_move node     
    def start(self):
        rate = rospy.Rate(self.sample_rate)
        img = []
        while not rospy.is_shutdown():
            img = self.update_map()
            cv2.imshow("Mapping - HW3", img)
            cv2.waitKey(1)
            cv2.imwrite('/home/rtk-noetic/rob_hw/src/mobile_robotics_hw/hw3/map/map.jpg', img*200)
            cv2.waitKey(2)

    # /base_scan subscriber
    def sub_laser(self, laser_data):
        self.laser = laser_data

    # /odom subscriber  
    def pose(self, pose_data):
        self.pose_data = pose_data
        # rospy.loginfo(f'subscribed {self.pose_data}')

    # data extraction from sensors readings
    def update_laser_scan(self):
        angs = np.array([]) # rad
        dists = np.array([]) # m
        info = np.array([]) # occupied / empty

        for i in range(len(self.laser.ranges)):
            # get angle
            ang = i * self.laser.angle_increment

            # get distance 
            # is distance more than max range
            if (self.laser.ranges[i] > self.laser.range_max):
                dist = self.laser.range_max
                inf = True
            elif (self.laser.ranges[i] < self.laser.range_min):
                dist = self.laser.range_min
                inf = True
            else:
                dist = self.laser.ranges[i]
                inf = False

            dists = np.append(dists, dist)
            angs = np.append(angs, ang)
            info = np.append(info, inf)

        return dists, angs, info
   
    # update the robot map based on the robot pose
    # world is at (0, 0)
    def update_map(self):
    
        # 1. Robot Location Pixel Values
        odomx, odomy = [self.pose_data.pose.pose.position.x, self.pose_data.pose.pose.position.y]   # x,y in [m]
        odomq = self.pose_data.pose.pose.orientation
        qlist = [ odomq.x, odomq.y, odomq.z, odomq.w]
        (_, _, odomtheta) = euler_from_quaternion(qlist)
        if odomtheta < 0:
            odomtheta = 2 * np.pi + odomtheta  # 0 to 2*pi
        xO, yO = self.grid.xy_pix(odomx, odomy)
        # print(xO, yO)
        # Robot motion pronts well
        # pdb.set_trace()
        # 2. Lidar Measurement Pixel Values
        distance, angle, dist_inf = self.update_laser_scan()
        dists_x = np.array([])
        dists_y = np.array([])
        # 2D transformation lidar values to world frame
        # project dist in x,y frame and append
        for (dist, ang) in zip(distance, angle):
            dists_x = np.append(dists_x, odomx + dist * np.cos(ang + odomtheta))
            dists_y = np.append(dists_y, odomy + dist * np.sin(ang + odomtheta))

        #3. Bresenham's algo to get Pixel Values form lidar scans
        # for BGR image of the grid map
        arrX = []
        arrY = []
        for (dist_x, dist_y, dist, inf) in zip(dists_x, dists_y, distance, dist_inf):
            # x2 and y2 pixel values for Bresenham's algorithm
            x2, y2 = self.grid.xy_pix(dist_x, dist_y)
            # line of free pixels, [robot position -> laser hit spot)
            for (x_bres, y_bres) in self.grid.get_laser_line(xO, yO, x2, y2):
                self.grid.update(x = x_bres, y = y_bres, p = self.min_p)
                # for BGR image of the grid map
                arrX.append(x2)
                arrY.append(y2)

            # mark laser reading as ocuppied
            if dist < self.laser.range_max and inf == False:
                self.grid.update(x = x2, y = y2, p = self.max_p)
                # for BGR image of the grid map
                arrX.append(x2)
                arrY.append(y2)
    
        # 3. Map all the probabilities    
        # converting grip map to BGR image
        bgr_image = self.grid.map_updated_grid(xO, yO, arrX, arrY)
        return bgr_image

if __name__ == "__main__":
    sf = mapping()

    # to begin subscribers without errors
    time.sleep(5)
    # Start spinning the node
    sf.start()
