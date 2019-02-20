#!/usr/bin/env python2

import rospy
import geometry_msgs.msg
import std_msgs.msg
import math


class Controller:
    takeoff = None
    land = None
    parrot_cmd = None
    camera_cmd = None
    rate = None
    empty_msg = None

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.takeoff = rospy.Publisher('/bebop/takeoff', std_msgs.msg.Empty, queue_size=1000)
        self.land = rospy.Publisher('/bebop/land', std_msgs.msg.Empty, queue_size=1000)
        self.parrot_cmd = rospy.Publisher('/bebop/cmd_vel', geometry_msgs.msg.Twist, queue_size=1000)
        self.camera_cmd = rospy.Publisher('/bebop/camera_control', geometry_msgs.msg.Twist, queue_size=1000)
        self.rate = rospy.Rate(10)
        self.empty_msg = std_msgs.msg.Empty()

    def do_takeoff(self):
        for i in xrange(3):
            self.takeoff.publish(self.empty_msg)

    def do_land(self):
        self.go_down(5)
        self.land.publish(self.empty_msg)

    def rotate(self, degree, sec):
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = math.radians(degree)
        self.publish_cmd(twist, sec)

    def go_forward(self, sec):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.1
        self.publish_cmd(twist, sec)

    def go_backward(self, sec):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = -0.1
        self.publish_cmd(twist, sec)

    def go_left(self, sec):
        twist = geometry_msgs.msg.Twist()
        twist.linear.y = 0.1
        self.publish_cmd(twist, sec)

    def go_right(self, sec):
        twist = geometry_msgs.msg.Twist()
        twist.linear.y = -0.1
        self.publish_cmd(twist, sec)

    def go_up(self, sec):
        twist = geometry_msgs.msg.Twist()
        twist.linear.z = 0.1
        self.publish_cmd(twist, sec)

    def go_down(self, sec):
        twist = geometry_msgs.msg.Twist()
        twist.linear.z = -0.1
        self.publish_cmd(twist, sec)

    def look_down(self):
        twist = geometry_msgs.msg.Twist()
        twist.angular.y = -200
        self.publish_camera_cmd(twist, 3)

    def look_up(self):
        twist = geometry_msgs.msg.Twist()
        twist.angular.y = 200
        self.publish_camera_cmd(twist, 3)

    def look_right(self):
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = 200
        self.publish_camera_cmd(twist, 3)

    def look_left(self):
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = -200
        self.publish_camera_cmd(twist, 3)

    def reset_camera(self):
        twist = geometry_msgs.msg.Twist()
        self.publish_camera_cmd(twist, 2)

    def publish_camera_twist(self, y=0, z=0):
        twist = geometry_msgs.msg.Twist()
        twist.angular.y = y
        twist.angular.z = z
        self.publish_camera_cmd(twist, 3)

    def publish_twist(self, x, y, z, degree, sec):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.z = math.radians(degree)
        self.publish_cmd(twist, sec)

    def publish_cmd(self, twist, sec):
        for i in xrange(sec * 10):
            self.parrot_cmd.publish(twist)
            self.rate.sleep()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.z = 0
        for i in xrange(5):
            self.parrot_cmd.publish(twist)

    def publish_camera_cmd(self, twist, sec):
        for i in xrange(sec):
            self.camera_cmd.publish(twist)
