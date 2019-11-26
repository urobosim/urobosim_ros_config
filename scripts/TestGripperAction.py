#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
import actionlib
from std_msgs.msg import String
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Transform


class Demo:

    def __init__(self):

        self.ph = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.ph.wait_for_server()
        self.ph2 = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.ph2.wait_for_server()


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

                pos = 0.8
                pos2 = 0.4
                pos3 = 0.0

                goal = Pr2GripperCommandGoal()
                goal.command.max_effort = 30
                goal.command.position = pos

                goal2 = Pr2GripperCommandGoal()
                goal2.command.max_effort = 30
                goal2.command.position = pos

                self.ph.send_goal(goal)
                self.ph2.send_goal_and_wait(goal2)

                goal = Pr2GripperCommandGoal()
                goal.command.max_effort = 30
                goal.command.position = pos2

                goal2 = Pr2GripperCommandGoal()
                goal2.command.max_effort = 30
                goal2.command.position = pos2

                self.ph.send_goal(goal)
                self.ph2.send_goal_and_wait(goal2)

                goal = Pr2GripperCommandGoal()
                goal.command.max_effort = 30
                goal.command.position = pos3

                goal2 = Pr2GripperCommandGoal()
                goal2.command.max_effort = 30
                goal2.command.position = pos3

                self.ph.send_goal(goal)
                self.ph2.send_goal_and_wait(goal2)

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
