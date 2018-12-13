#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import string
from math import cos, sin, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from getodomdata_serial.msg import mobileRobot_msgs

global callback_flag, left_enc, right_enc, dright, dleft
callback_flag = 0
left_enc = 0
right_enc = 0
dright = 0
dleft = 0

def encoder_callback(msg):
    global callback_flag, left_enc, right_enc
    left_enc = msg.left_enc
    right_enc = msg.right_enc
    rospy.loginfo('left_enc=%d,right_enc=%d' % (left_enc, right_enc))
    callback_flag = 1

def talker():
    rospy.init_node('serial_get_odomdata', anonymous=False)
    node_name = rospy.get_name()
    rospy.loginfo("%s is starting..." % node_name)

    # initial parameters
    # bad_encoder_count = 0
    # now = rospy.Time.now()
    then = rospy.Time.now()
    # enc_left = 0
    # enc_right = 0
    D = 0.14
    wheel_track = 0.5
    global callback_flag, left_enc, right_enc, dright, dleft
    x = 0
    y = 0
    th = 0
    get_encode_count = 0
    base_frame = 'base_link'

    # initial arguments

    # subscribe/publish
    odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
    rospy.Subscriber("/encoder", mobileRobot_msgs, encoder_callback)


    odomBroadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # try:
        #     left_enc, right_enc = getodomdata.odomdataprocess()
        #     rospy.loginfo('left_enc=%d,right_enc=%d' % (left_enc, right_enc))
        #     now = rospy.Time.now()
        # except:
        #     bad_encoder_count += 1
        #     rospy.logerr("Encoder exception count: " + str(bad_encoder_count))
        #     return
        if callback_flag == 1:
            now = rospy.Time.now()
            # record get encode left&right in first time, must initial enc_right&left
            get_encode_count += 1
            if get_encode_count == 1:
                enc_right = right_enc
                enc_left = left_enc
            dt = now - then
            then = now
            dt = dt.to_sec()

            # Calculate odometry
            if enc_left is None:
                dright = 0
                dleft = 0
            else:
                # (4096 * 7.5) dright is right boot position
                dright = (right_enc - enc_right) * pi * D / (4096 * 7.5)
                # dleft is left boot positon
                dleft = (left_enc - enc_left) * pi * D / (4096 * 7.5)

            enc_right = right_enc
            enc_left = left_enc
            
            # get center point move distance
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / wheel_track  #get center point move theta
            vxy = dxy_ave / dt   #center point velocity
            vth = dth / dt      #center point rotate speed
            rospy.loginfo('dxy_ave = %f', dxy_ave)
            if dxy_ave != 0:
                dx = cos(dth) * dxy_ave   # in base_link x-axis move distance
                dy = -sin(dth) * dxy_ave  # in base_link y_axis move distance
                x += (cos(th) * dx - sin(th) * dy)  #transfer x-axis move distance from base_link-axis to odom-axis
                y += (sin(th) * dx + cos(th) * dy)  #transfer y-axis move distance from base_link-axis to odom-axis

            if dth != 0:
                th += dth
            rospy.loginfo('x = %f, y = %f' % (x, y))
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
                base_frame,  # sub-ordi
                "odom"  # farthe-ordi
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

            callback_flag = 0

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
