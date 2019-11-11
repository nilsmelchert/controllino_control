#!/usr/bin/env python2

import rospy
from rosserial_arduino.srv import Test, TestResponse

def callback(req):
    controllino_status = rospy.get_param('/controllino_status')
    controllino_status[req.input[:3]] = int(req.input[3])
    rospy.set_param('/controllino_status', controllino_status)
    return TestResponse("")

if __name__ == '__main__':
    rospy.init_node("controllino_ss_nd", log_level=rospy.INFO, anonymous=False)
    rospy.loginfo("Starting controllino getter node")
    rospy.set_param('/controllino_status', {'a08': 0, 'a09': 0, 'a10': 0, 'a11': 0, 'a12': 0, 'a13': 0, 'a14': 0, 'a15': 0})
    s = rospy.Service('controllino_getter', Test, callback)
    rospy.spin()
