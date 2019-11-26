#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
import actionlib
from std_msgs.msg import String
from urobosim_msgs.msg import PerceiveObjectAction, PerceiveObjectGoal
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Transform


class Demo:

    def __init__(self):

        self.ph = actionlib.SimpleActionClient('', PointHeadAction)
        self.ph.wait_for_server()

        self.published = False
        # self.pub = rospy.Publisher('/head_traj_controller/head_action/goal',
        #                                  PointHeadActionGoal,
        #                                  queue_size=1)
        self.run_demo()

    def DemoCommandsCallback(self, data):
        rospy.sleep(2.0)
        self.run_demo()


    def run_demo(self):
        try:
            if not self.published:
                point = PointStamped()
                point.header.frame_id = 'base_footprint'
                point.point.x = 1.44
                point.point.y = 0.79
                point.point.z = 0.85

                goal = PointHeadGoal()
                goal.pointing_frame = "high_def_frame"
                goal.pointing_axis.x = 1
                goal.pointing_axis.y = 0
                goal.pointing_axis.z = 0
                goal.target = point

                self.ph.send_goal_and_wait(goal)
                print goal
                rospy.sleep(1.)
                self.published = True
                rospy.sleep(0.1)
        except KeyboardInterrupt:
            print('interrupted!')
            sys.exit(0)



if __name__ == '__main__':
    rospy.init_node('PR2Demo', anonymous=True)
    try:
        d = Demo()
    except rospy.ROSInterruptException:
        print "Shutdown simulator demo"
