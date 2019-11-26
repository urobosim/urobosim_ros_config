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
        self.pose0 = {'torso_lift_joint': 0.20}

        self.pose3 = {'r_shoulder_pan_joint': 0.30,
                     'r_shoulder_lift_joint': 0.0,
                     'r_upper_arm_roll_joint': -1.95,
                     'r_elbow_flex_joint': -0.55,
                     # 'r_forearm_roll_joint': 1.76632,
                     # 'r_wrist_flex_joint': -0.10001,
                     'r_wrist_roll_joint': -1.55,

                     'l_shoulder_pan_joint': 0.25,
                     'l_shoulder_lift_joint': 1.05,
                     'l_upper_arm_roll_joint': 1.75,
                     'l_elbow_flex_joint': - 1.75,
                     # 'l_forearm_roll_joint': 16.99,
                     # 'l_wrist_flex_joint': - 0.10001,
                     'l_wrist_roll_joint': 0.}
        # self.default_pose = {'r_shoulder_pan_joint': -1.7125,
        #              'r_shoulder_lift_joint': -0.25672,
        #              'r_upper_arm_roll_joint': -1.46335,
        #              'r_elbow_flex_joint': -2.12216,
        #              'r_forearm_roll_joint': 1.76632,
        #              'r_wrist_flex_joint': -0.10001,
        #              'r_wrist_roll_joint': 0.05106,
        #
        #              'l_shoulder_pan_joint': 1.9652,
        #              'l_shoulder_lift_joint': - 0.26499,
        #              'l_upper_arm_roll_joint': 1.3837,
        #              'l_elbow_flex_joint': - 2.1224,
        #              'l_forearm_roll_joint': 16.99,
        #              'l_wrist_flex_joint': - 0.10001,
        #              'l_wrist_roll_joint': 0}


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
            # Pose.header.frame_id = l_tip
            # Pose.header.stamp = rospy.get_rostime()
            # Pose.pose.position = Point(0.80, -0.3, 0.5)
            # O= quaternion_from_euler(0.0, 0.0, 0.0)
            # Pose.pose.orientation = Quaternion(O[0], O[1], O[2], O[3])
            # self.giskard.set_cart_goal( u'base_footprint',l_tip , Pose)
            # self.giskard.plan_and_execute()



            # Pose = PoseStamped()
            # Pose.header.frame_id = l_tip
            # Pose.header.stamp = rospy.get_rostime()
            # Pose.pose.position = Point(0.586, 0.007, 0.515)
            #
            # O= quaternion_from_euler(0.0, 0.0, 0.7)
            # Pose.pose.orientation = Quaternion(O[0], O[1], O[2], O[3])
            # Pose.pose.orientation = Quaternion(-0.058, 0.229, 0.507, 0.829)
            # self.giskard.set_cart_goal( u'base_footprint',l_tip , Pose)
            # self.giskard.set_rotation_goal( u'base_footprint',l_tip , Pose)

            # Pose2 = PoseStamped()
            # Pose2.header.frame_id = u'r_elbow_flex_link'
            # Pose2.header.stamp = rospy.get_rostime()
            # Pose2.pose.position = Point(0.586, 0.007, 0.415)
            # self.giskard.set_joint_goal(self.pose1)
            # self.giskard.plan_and_execute()
            #
            # print "between"
            #
            #
            # self.giskard.set_joint_goal(self.pose3)
            # self.giskard.plan_and_execute()
            # self.giskard.set_joint_goal(self.pose1)
            # self.giskard.plan_and_execute()


            self.giskard.plan_and_execute()
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
