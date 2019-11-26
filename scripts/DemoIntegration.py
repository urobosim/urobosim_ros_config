#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
from giskardpy.python_interface import GiskardWrapper
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from geometry_msgs.msg import Transform
from u_robo_sim_communication.srv import SimulationCommands
import tf2_ros
from tf.transformations import *
import tf
# import math
# import geometry_msgs.msg


class Demo:

    def __init__(self):
        self.stage = 0
        self.giskard = GiskardWrapper()
        self.giskard.allow_all_collisions()
        self.pose0 = {'torso_lift_joint': 0.15}


        self.vel_pub = rospy.Publisher('/base_controller/command',
                                         Twist,
                                         queue_size=1)

        # self.DemoCommandsSub = rospy.Subscriber("/DemoCommands",
        #                                         String,
        #                                         self.DemoCommandsCallback,
        #                                         queue_size = 1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # print "start waiting"
        # rospy.wait_for_service('SimulationCommands')
        # print "ready"
        self.run_demo()

    def run_demo(self):
        try:
            print "stage 0"
            self.giskard.set_joint_goal(self.pose0)
            # self.giskard.add_cmd()

            l_tip = u'r_gripper_tool_frame'

            self.giskard.plan_and_execute()
            print "stage 1"
            rospy.sleep(0.5)
            self.stage = -1
        except KeyboardInterrupt:
            print('interrupted!')




if __name__ == '__main__':
    rospy.init_node('PR2Demo', anonymous=True)
    d = Demo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutdown simulator demo"
