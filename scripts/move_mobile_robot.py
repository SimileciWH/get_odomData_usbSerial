#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from getodomdata_serial.msg import mobileRobot_velocity


class MoveMobileRobot():

    def __init__(self):
        rospy.init_node('move_mobile_robot', anonymous=False)
        self.node_name = rospy.get_name()
        rospy.loginfo("%s is started..." % self.node_name)

        # init my parameters
        self.dx = 0
        self.dr = 0
        self.left_vel = 0
        self.right_vel = 0

        # init my arguments
        self.wheel_dist = rospy.get_param("~wheel_distance", default=0.2)
        self.rate = rospy.get_param("~rate", default=10)   # 10 hz
        # subscribe/publish
        rospy.Subscriber("/robot0/cmd_vel", Twist, self.twist_callback)
        self.wheel_vtarget_pub = rospy.Publisher("/wheel_vtarget", mobileRobot_velocity, queue_size=1000)

    def twist_callback(self, data):
        self.dx = data.linear.x
        self.dr = data.angular.z
        rospy.loginfo("linear x = %f, angular z = %f" % (self.dx, self.dr))

    def calculate_pub(self):
        self.left_vel = self.dx - self.dr * self.wheel_dist / 2
        self.right_vel = self.dx + self.dr * self.wheel_dist / 2
        rospy.loginfo("the left_vel = %f, right_vel = %f" % (self.left_vel, self.right_vel))
        return self.left_vel, self.right_vel

    def spin(self):
        # main loop
        msg = mobileRobot_velocity()
        r = rospy.Rate(self.rate)   # 1 sec loop rate times ==> HZ
        while not rospy.is_shutdown():
            msg.left_vtarget, msg.right_vtarget = self.calculate_pub()
            msg.name1 = "left_vtarget"
            msg.name2 = "right_vtarget"
            self.wheel_vtarget_pub.publish(msg)
            r.sleep()


if __name__ == '__main__':
    try:
        move_mobile_robot = MoveMobileRobot()
        move_mobile_robot.spin()
    except rospy.ROSInterruptException:
        pass
