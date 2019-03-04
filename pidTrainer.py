#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped

def main():
        rospy.init_node('pid_trainer')

        dbw_enabled_pub = rospy.Publisher('/vehicle/dbw_enabled', Bool, queue_size=1)
        dbw_twist_cmd_pub = rospy.Publisher('/twist_cmd', TwistStamped, queue_size=1)
        speed = 0.0
        reset_time = rospy.get_time()

        while not rospy.is_shutdown():
            cur_time = rospy.get_time()

            if speed == 0.0 and cur_time-reset_time > 5.0:
                speed = 11.0
                reset_time = rospy.get_time()
            if speed == 11.0 and cur_time-reset_time > 20.0:
                speed = 0.0
                reset_time = rospy.get_time()

            current_timestamp = rospy.get_rostime()

            dbw_enabled = Bool()
            dbw_enabled.data = True
            dbw_enabled_pub.publish(dbw_enabled)

            dbw_twist_cmd = TwistStamped()
            dbw_twist_cmd.header.stamp = current_timestamp
            dbw_twist_cmd.header.frame_id = ""
            dbw_twist_cmd.twist.linear.x = speed
            #dbw_twist_cmd.twist.angular.z = 0.0
            dbw_twist_cmd_pub.publish(dbw_twist_cmd)
            

if __name__ == '__main__':

    main()
