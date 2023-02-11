#! /usr/bin/env python3
# This ROS node computes a potential field sum based on the laser scan readings
# The code drives the robot forwards at a constant speed,
# The apf planner turns the robot to avoid obstacles as it detects them.

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np

class move:
    def __init__(self, tp = 'simple_move'):
        # Initialize the node, and call it "apf".
        rospy.init_node(tp)
        # Set up a publisher.  The default topic for Twist messages is cmd_vel.
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Set up a subscriber.  The default topic for LaserScan messages is base_scan.
        self.subscriber = rospy.Subscriber('base_scan', LaserScan, self.sub_laser)

        # Set up a subscriber to get robot's live location in the world
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.pose)

        # system safety constraints
        self.min_linear = 0.01
        self.min_angular = 0.1

        # get parameters
        self.goal = Point()
        self.goal.x = rospy.get_param('~gx')
        self.goal.y = rospy.get_param('~gy') 
        self.max_linear = rospy.get_param('~v_max')
        self.max_angular = rospy.get_param('~w_max')

        self.robot_radius = rospy.get_param('~rr')
        self.siz_obs = rospy.get_param('~obs_sz')
        self.max_angle = rospy.get_param('~max_angle')
        self.sample_rate = rospy.get_param('~sample_rate')
        self.range_lim = rospy.get_param('~range_lim')
        self.drive_scale = rospy.get_param('~drive_scale')
        self.turn_scale = rospy.get_param('~turn_scale')
        
        # laser readings
        self.laser = None
        self.pose_data = None

    # start spinning the node
    def start(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            self.get_potential()
            rate.sleep()

    # calculate the net potenial gradient to move the robot
    def get_potential(self):
        if self.laser == None:
            return

        t = Twist()
        laser_data = self.laser
        ranges = laser_data.ranges

        # get repulsive potential gradients
        t_pot = np.array([0.0,0.0])
        pot = np.array([0.0, 0.0])
        for i in range(len(ranges)):
            if ranges[i] <= self.range_lim:
                # x, y component of sensor reading in the body frame
                ox = ranges[i] * np.cos(i)
                oy = ranges[i] * np.sin(i)
                pot = self.calc_repulsive_del(ox, oy)
                t_pot += pot

        # get attractive potential gradients       
        t_pot += self.calc_attractive_del()

        # get desired velocity command from the net gradient
        self.vel_from_force(t, t_pot)

    def calc_attractive_del(self):
        X1 = self.pose_data.x - self.goal.x
        Y1 = self.pose_data.y- self.goal.y
        # calc attractive potential grad
        # calculating the distance asumming the goal is at Origin
        df = np.sqrt(X1**2 + Y1**2)

        # calCulating the Theta
        theta = np.arctan2(Y1,X1)

        # parameters defining the potential gradient function wrt distance from the goal
        s = 7

        # stopping distance
        goal_sz = 0.05
        # Using the equations given in the class
        if df < 0.1:
            delx = 0
            dely = 0
        elif df > goal_sz+s:
            delx = 200* s *np.cos(theta)
            dely = 200 * s *np.sin(theta)
        else:
            delx = 200 * (df-goal_sz) *np.cos(theta)
            dely = 200 * (df-goal_sz) *np.sin(theta)

        return np.array([delx, dely])

    def calc_repulsive_del(self, ox, oy):
        X1 = ox
        Y1 = oy
        # calc repulsive potential grad
        # distance from obs
        df= np.sqrt(X1**2 + Y1**2)

        # calculating the Theta
        theta = np.arctan2(Y1,X1)

        # upper bound of influence
        s = 7

        # Using the equations given in the class
        if df < 0.5:
            delx = -5 * np.sign(np.cos(theta))
            dely = -5 * np.sign(np.cos(theta))
        elif df > self.siz_obs+s:
            delx = -5 * (s) *np.cos(theta)
            dely = -5 * (s) *np.sin(theta)
        else:
            delx = -150 * (s+df-self.siz_obs) *np.cos(theta)
            dely = -150 * (s+df-self.siz_obs) *np.sin(theta)

        return np.array([delx, dely])
        
    def vel_from_force(self, t_vel, tot_f):
        # velocity control command generation - open loop
        t_vel.linear.x = tot_f[0] * self.drive_scale
        t_vel.angular.z = np.arctan2(tot_f[1],tot_f[0]) * self.turn_scale

        # clipping velocity for safety
        if abs(t_vel.linear.x ) < self.min_linear:
            t_vel.linear.x  = 0
        if abs(t_vel.angular.z) < self.min_angular:
            t_vel.angular.z = 0

        if t_vel.linear.x  < -self.max_linear:
            t_vel.linear.x  = -self.max_linear
        elif t_vel.linear.x  > self.max_linear:
            t_vel.linear.x  = self.max_linear
        
        if t_vel.angular.z < -self.max_angular:
            t_vel.angular.z = -self.max_angular
        elif t_vel.angular.z > self.max_angular:
            t_vel.angular.z = self.max_angular

        self.publish_cmdvel(t_vel)

    def sub_laser(self, laser_data):
        self.laser = laser_data
        # Range data at 10 deg:  laser_data.ranges[10]
    
    # get odom data
    def pose(self, pose_data):
        self.pose_data = pose_data.pose.pose.position

    def publish_cmdvel(self, t):
        self.publisher.publish(t)

        # Print out a log message to the INFO channel to let us know it's working.
        rospy.loginfo(f'Goal {self.goal}')
        rospy.loginfo(f'Published {t.linear.x, t.angular.z}')

if __name__ == "__main__":
    sf = move('apf')
    sf.start()