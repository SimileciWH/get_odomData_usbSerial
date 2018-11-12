#!/usr/bin/env python


import time
import string
from math import *


import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


import getodomdata


global bad_encoder_count
global now,then
global enc_left, enc_right
global D
global pi
global wheel_track
global x,y
global th


def talker():
    odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
    rospy.init_node('serial_get_odomdata',anonymous=False)

    odomBroadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(5)

    bad_encoder_count = 0
    now = rospy.Time.now()
    then = rospy.Time.now()
    enc_left = 0
    enc_right = 0
    D = 0.14
    pi = 3.14
    wheel_track = 0.5
    x = 0
    y = 0
    th = 0
    base_frame = 'base_link'

    while not rospy.is_shutdown():
        dright = 0
        dleft = 0

        try:
            left_enc, right_enc = getodomdata.odomdataprocess()
            rospy.loginfo('left_enc=%d,right_enc=%d' % (left_enc,right_enc))
            now = rospy.Time.now()
        except:
            bad_encoder_count += 1
            rospy.logerr("Encoder exception count: " + str(bad_encoder_count))
            return

        dt = now - then
        then = now
        dt = dt.to_sec()

        # Calculate odometry
        if enc_left == None:
            dright = 0
            dleft = 0
        else:
            # (4096 * 7.5) dright is right boot position
            dright = (right_enc - enc_right) * pi * D / (4096*7.5)
            #dleft is left boot positon
            dleft = (left_enc - enc_left) * pi * D / (4096*7.5)


        enc_right = right_enc
        enc_left = left_enc

        dxy_ave = (dright + dleft) / 2.0
        dth = (dright - dleft) / wheel_track
        vxy = dxy_ave / dt
        vth = dth / dt

        if (dxy_ave != 0):
            dx = cos(dth) * dxy_ave
            dy = -sin(dth) * dxy_ave
            x += (cos(th) * dx - sin(th) * dy)
            y += (sin(th) * dx + cos(th) * dy)

        if (dth != 0):
            th += dth

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(th / 2.0)
        quaternion.w = cos(th / 2.0)

        # Create the odometry transform frame broadcaster.
        odomBroadcaster.sendTransform(
            (x, y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            base_frame , # sub-ordi
        "odom" # farthe-ordi
        )

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = base_frame
        odom.header.stamp = now
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vxy
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth

        odomPub.publish(odom)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
