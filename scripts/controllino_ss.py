#!/usr/bin/env python

import rospy
from rosserial_arduino.srv import Test, TestResponse

def callback(req):
    print(req.input)
    return TestResponse("")

if __name__ == '__main__':
    rospy.init_node("controllino_ss_nd")
    s = rospy.Service('controllino_getter', Test, callback)
    rospy.spin()
