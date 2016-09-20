#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

import rospy
from mavros.srv import SetMode

def set_mode(mode):
    try:
        setmode_cl = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        
        ret = setmode_cl(base_mode=0, custom_mode=mode)
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        rospy.loginfo("SET MODE Request failed.")
    else:
        rospy.loginfo("SET MODE Request success")


def rc_override_control():
    
    rospy.init_node("mavteleop")
    rospy.loginfo("MAV-Teleop: RC Override control type.")
    
    while(1):
	    set_mode("GUIDED")
        


def main():
    rc_override_control()


if __name__ == '__main__':
    main()

