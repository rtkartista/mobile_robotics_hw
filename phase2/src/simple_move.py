#! /usr/bin/env python3
# The robot should ignore obstacles to the side.
# Only stops for stuff directly in front of it.

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
class move:
    def __init__(self, tp = 'simple_move'):
        # Initialize the node, and call it "simple_move".
        rospy.init_node('simple_move')

        # Set up a publisher.  The default topic for Twist messages is cmd_vel.
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Set up a subscriber.  The default topic for LaserScan messages is base_scan.
        self.subscriber = rospy.Subscriber('scan', LaserScan, self.sub_laser, queue_size=5)
        
        # Set up a subscriber to get robot's live location in the world
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.pose, queue_size=5)

        # get parameters
        self.max_linear = rospy.get_param('~v_max')
        self.max_angle = rospy.get_param('~max_angle')
        self.sample_rate = rospy.get_param('~sample_rate')
        self.range_lim = rospy.get_param('~range_lim')

        # laser readings
        self.laser = None
        self.pose_data = None

        
    # start spinning the simple_move node     
    def start(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            self.detect_obs()
            rate.sleep()

    # detect obstacles in front of the vehicle and stop
    def detect_obs(self):
        if self.laser == None:
            return

        # get laser scan distances
        ranges = np.array(self.laser.ranges)

        t = Twist()
        if ranges[0]>self.range_lim and ranges[self.max_angle]>self.range_lim and ranges[-self.max_angle]>self.range_lim: # Checks if there are obstacles in front
            t.linear.x = self.max_linear
            t.angular.z = 0.0
        else:
            t.linear.x = 0.0
            t.angular.z = self.max_linear
            if ranges[0]>self.range_lim and ranges[self.max_angle]>self.range_lim and ranges[-self.max_angle]>self.range_lim: # Check for clearance after turning
                t.linear.x = self.max_linear
                t.angular.z = 0.0

        # publish the velocity to the sim
        self.publish_cmdvel(t)

    # /base_scan subscriber
    def sub_laser(self, laser_data):
        self.laser = laser_data

    # /odom subscriber  
    def pose(self, pose_data):
        self.pose_data = pose_data
        # rospy.loginfo(f'Published {pose_data.pose}')

    # /cmd_vel publisher
    def publish_cmdvel(self, t):
        # rospy.loginfo(f'Published {t.linear.x}')
        self.publisher.publish(t)


if __name__ == "__main__":
    sf = move()

    # Start spinning the node
    sf.start()