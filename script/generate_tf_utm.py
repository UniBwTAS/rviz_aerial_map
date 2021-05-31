#!/usr/bin/env python
import utm
import rospy
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def callback(fix_msg):
    east, north, zone_number, zone_letter = utm.from_latlon(fix_msg.latitude, fix_msg.longitude)
    tf_msg = TransformStamped()
    tf_msg.header.stamp = fix_msg.header.stamp
    tf_msg.header.frame_id = "utm"
    tf_msg.child_frame_id = "base_link"
    tf_msg.transform.translation.x = east
    tf_msg.transform.translation.y = north
    tf_msg.transform.translation.z = fix_msg.altitude
    tf_msg.transform.rotation.x = 0
    tf_msg.transform.rotation.y = 0
    tf_msg.transform.rotation.z = 0
    tf_msg.transform.rotation.w = 1
    tf_broadcaster.sendTransform([tf_msg])


rospy.init_node('utm_tf', anonymous=True)
rospy.Subscriber("gps/fix", NavSatFix, callback)

tf_broadcaster = TransformBroadcaster()

rospy.spin()
