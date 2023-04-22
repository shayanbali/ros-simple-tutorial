#!usr/bin/python3

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians 


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=False)

        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.linear_speed = 0.2 # m/s
        self.angular_speed = 0.2 # m/s
        self.goal_angle = radians(90) # rad
        self.stop_dist = 1 # m
        

        self.go , self.rotate = 0 , 1
        self.state = self.go


    def laser_callback(self, msg:LaserScan):
        if msg.ranges[0] <= self.stop_dist:
            self.state = self.rotate


    def get_heading():
        pose = 

