#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from mtg_mtsp_task_allocator.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    print("Waiting for service...")
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    rospy.init_node("ta_node")
    rate = rospy.Rate(10)
    #while not rospy.is_shutdown():

    #Client
    req = "ta"
    rospy.wait_for_service("ta_out")
    ta_out_dummy = rospy.ServiceProxy("ta_out", ta_out)
    resp = ta_out_dummy(req)
    print(resp)
    rospy.spin()
