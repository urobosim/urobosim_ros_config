#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
from giskardpy.python_interface import GiskardWrapper
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
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
        # self.Test_Pose = {'torso_lift_joint': 0.2}
        self.default_pose = {'r_shoulder_pan_joint': -0.25,
                     'r_shoulder_lift_joint': 0.525,
                     'r_upper_arm_roll_joint': -1.75,
                     'r_elbow_flex_joint': -1.75,
                     # 'r_forearm_roll_joint': 1.76632,
                     # 'r_wrist_flex_joint': -0.10001,
                     # 'r_wrist_roll_joint': 0.05106,

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

        self.grasp_prep_pose = {'r_shoulder_pan_joint': -0.59,
                     'r_shoulder_lift_joint': -0.0,
                     'r_upper_arm_roll_joint': -1.57,
                     'r_elbow_flex_joint': -1.55,
                     'r_forearm_roll_joint': 0.,
                     'r_wrist_flex_joint': 0.,
                     'r_wrist_roll_joint': 1.57,

                     'l_shoulder_pan_joint': 0.25,
                     'l_shoulder_lift_joint': 1.05,
                     'l_upper_arm_roll_joint': 1.75,
                     'l_elbow_flex_joint': - 1.75,
                     'l_wrist_roll_joint': 0,
                     'head_tilt_joint': 0.35,
                     'torso_lift_joint': 0.20}

        self.grasp_pose = {'r_shoulder_pan_joint': -0.29,
                     'r_shoulder_lift_joint': -0.0,
                     'r_upper_arm_roll_joint': -1.57,
                     'r_elbow_flex_joint': -1.25,
                     'r_forearm_roll_joint': 0.,
                     'r_wrist_flex_joint': 0.,
                     'r_wrist_roll_joint': 1.57,

                     'l_shoulder_pan_joint': 0.25,
                     'l_shoulder_lift_joint': 1.05,
                     'l_upper_arm_roll_joint': 1.75,
                     'l_elbow_flex_joint': - 1.75,
                     'l_wrist_roll_joint': 0,
                     'head_tilt_joint': 0.35,
                     'torso_lift_joint': 0.20}

        self.after_grasp_pose = {'r_shoulder_pan_joint': -0.29,
                     'r_shoulder_lift_joint': -0.0,
                     'r_upper_arm_roll_joint': -1.57,
                     'r_elbow_flex_joint': -1.25,
                     'r_forearm_roll_joint': 0.,
                     'r_wrist_flex_joint': 0.,
                     'r_wrist_roll_joint': 1.57,

                     'l_shoulder_pan_joint': 0.25,
                     'l_shoulder_lift_joint': 1.05,
                     'l_upper_arm_roll_joint': 1.75,
                     'l_elbow_flex_joint': - 1.75,
                     'l_wrist_roll_joint': 0,
                     'head_tilt_joint': 0.35,
                     'torso_lift_joint': 0.29}

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

    def DemoCommandsCallback(self, data):
        rospy.sleep(2.0)
        self.run_demo()

    def CalcLinearAndAngular(self, target_pose, max_vel = 0.1, error_threshold = 0.01):

        angl_vel_mod = 6
        lin_vel_mod = 4


        now = time.time()
        trans = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
        print now - trans.header.stamp.to_sec()
        # print now, trans.header.stamp.secs,  trans.header.stamp.nsecs/1000.
        x = target_pose.translation.x - trans.transform.translation.x
        y = target_pose.translation.y - trans.transform.translation.y

        vel = [x, y, 0., 0.]

        q_cur = trans.transform.rotation
        q_cur= [q_cur.x, q_cur.y, q_cur.z, q_cur.w]
        q_cur_inv = [q_cur[0], q_cur[1], q_cur[2], -q_cur[3]]


        result = quaternion_multiply(quaternion_multiply(q_cur_inv, vel),quaternion_conjugate(q_cur_inv))


        q_traget = target_pose.rotation
        # q_traget = [q_traget.x, q_traget.y, q_traget.z, q_traget.w]

        rotation = tf.transformations.quaternion_multiply(q_traget, q_cur_inv)
        angle = 2 * math.acos(rotation[3])
        if angle > math.pi:
            angle = 2.0 * math.pi - angle

        direction = np.sign(rotation[2])
        angle *= direction
        # print rotation, angle
        msg = Twist()

        if np.abs(angle) > 0.06 * angl_vel_mod:
            msg.angular.z = direction * 0.06 * angl_vel_mod
        elif np.abs(angle) < 0.01 * angl_vel_mod:
            msg.angular.z = 0.0
        else:
            msg.angular.z = angle

        msg.linear.x = result[0]
        msg.linear.y = result[1]
        # msg.linear.x = x
        # msg.linear.y = y
        lin_vel = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2)
        if lin_vel > max_vel * lin_vel_mod:
            vel_correction = max_vel / lin_vel
            msg.linear.x *= vel_correction * lin_vel_mod
            msg.linear.y *= vel_correction * lin_vel_mod
        elif lin_vel < error_threshold * lin_vel_mod:
            msg.linear.x = 0.
            msg.linear.y = 0.

        return msg

    def run_demo(self):
        while self.stage != 13:
            try:
                if self.stage == 0:
                    print "stage 0"
                    # self.giskard.set_joint_goal(self.Test_Pose)
                    self.giskard.set_joint_goal(self.default_pose)
                    self.giskard.plan_and_execute()
                    rospy.sleep(0.5)
                    self.stage = 1

                elif self.stage == 1:
                    print "stage 1"
                    target_pose = Transform()
                    target_pose.translation.x = -2.47
                    target_pose.translation.y = -0.31
                    target_pose.rotation = quaternion_from_euler(0, 0, -math.pi/2.)
                    vel_msg = self.CalcLinearAndAngular(target_pose)
                    self.vel_pub.publish(vel_msg)
                    if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                        self.stage = 2
                elif self.stage == 2:
                    print "stage 2"
                    target_pose = Transform()
                    target_pose.translation.x = -2.53
                    target_pose.translation.y = -1.25
                    target_pose.rotation = quaternion_from_euler(0, 0, -math.pi/2.0)
                    vel_msg = self.CalcLinearAndAngular(target_pose)
                    self.vel_pub.publish(vel_msg)
                    if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                        self.stage = 3
                elif self.stage == 3:
                    print "stage 3"
                    target_pose = Transform()
                    target_pose.translation.x = -2.53
                    target_pose.translation.y = -1.25
                    target_pose.rotation = quaternion_from_euler(0, 0, -math.pi)
                    vel_msg = self.CalcLinearAndAngular(target_pose)
                    self.vel_pub.publish(vel_msg)
                    if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                        self.stage = 4
                elif self.stage == 4:
                    print "stage 4"
                    self.giskard.set_joint_goal(self.grasp_prep_pose)
                    self.giskard.plan_and_execute()
                    rospy.sleep(0.5)
                    self.stage = 5
                elif self.stage == 5:
                    print "stage 5"
                    target_pose = Transform()
                    target_pose.translation.x = -3.37
                    target_pose.translation.y = -1.15
                    target_pose.rotation = quaternion_from_euler(0, 0, -math.pi)
                    vel_msg = self.CalcLinearAndAngular(target_pose)
                    self.vel_pub.publish(vel_msg)
                    if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                        self.stage = 6
                elif self.stage == 6:
                    print "stage 6"
                    self.giskard.set_joint_goal(self.grasp_pose)
                    self.giskard.plan_and_execute()
                    rospy.sleep(0.5)
                    self.stage = 7
                elif self.stage == 7:
                    print "stage 7"
                    result = rospy.ServiceProxy('SimulationCommands', SimulationCommands)
                    result("GRASP")
                    self.stage = 8
                elif self.stage == 8:
                    print "stage 8"
                    self.giskard.set_joint_goal(self.after_grasp_pose)
                    self.giskard.plan_and_execute()
                    rospy.sleep(0.5)
                    self.stage = 9
                elif self.stage == 9:
                    print "stage 9"
                    target_pose = Transform()
                    target_pose.translation.x = -3.17
                    target_pose.translation.y = -1.15
                    target_pose.rotation = quaternion_from_euler(0, 0, 0.)
                    vel_msg = self.CalcLinearAndAngular(target_pose)
                    self.vel_pub.publish(vel_msg)
                    if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                        self.stage = 10
                elif self.stage == 10:
                    print "stage 10"
                    target_pose = Transform()
                    target_pose.translation.x = -1.9
                    target_pose.translation.y = -1.45
                    target_pose.rotation = quaternion_from_euler(0, 0, 0)
                    vel_msg = self.CalcLinearAndAngular(target_pose)
                    self.vel_pub.publish(vel_msg)
                    if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                        self.stage = 11
                elif self.stage == 11:
                    print "stage 11"
                    result = rospy.ServiceProxy('SimulationCommands', SimulationCommands)
                    result("RELEASE")
                    self.stage = 12
                elif self.stage == 12:
                    print "stage 12"
                    target_pose = Transform()
                    target_pose.translation.x = -2.3
                    target_pose.translation.y = -1.45
                    target_pose.rotation = quaternion_from_euler(0, 0, 0)
                    vel_msg = self.CalcLinearAndAngular(target_pose)
                    self.vel_pub.publish(vel_msg)
                    if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                        self.stage = 13
                rospy.sleep(0.01)
            except KeyboardInterrupt:
                print('interrupted!')



if __name__ == '__main__':
    rospy.init_node('PR2Demo', anonymous=True)
    try:
        d = Demo()
    except KeyboardInterrupt:
        print "Shutdown simulator demo"
