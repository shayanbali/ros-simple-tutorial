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
        self.epsilon = 0.0001
        

        self.go , self.rotate = 0 , 1
        self.state = self.go


    def laser_callback(self, msg:LaserScan):
        if msg.ranges[0] <= self.stop_dist:
            self.state = self.rotate


    def get_heading(self):
        msg = rospy.wait_for_message("/odom", Odometry)
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        return yaw
    

    def run(self):

        while not rospy.is_shutdown():
            if self.state == self.go:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
                continue
                
            self.cmd_publisher(Twist())
            rospy.sleep(1)

            remaining = self.goal_angle
            prev_angle = self.get_heading()

            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_publisher(twist)


            while remaining >= self.epsilon:
                curren_angle = self.get_heading
                delta = abs(prev_angle - curren_angle)
                remaining -= delta

            self.cmd_publisher.publish(Twist())

            rospy.sleep(1)

            self.state = self.go

