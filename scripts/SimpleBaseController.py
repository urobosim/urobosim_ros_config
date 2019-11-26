#! /usr/bin/env python

import rospy
import actionlib

import math
import time
import numpy as np

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Transform

import tf2_ros
from tf.transformations import *
import tf

import move_base_msgs.msg


class SimulationBaseController(object):
    _feedback = move_base_msgs.msg.MoveBaseFeedback()
    _result = move_base_msgs.msg.MoveBaseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer("nav_pcontroller/move_base", move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.vel_pub = rospy.Publisher('/base_controller/command',
                                         Twist,
                                         queue_size=1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def execute_cb(self, goal):

        # helper variables
        r = rospy.Rate(100)
        success = False

        # vector = (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z)
        target_pose = Transform(goal.target_pose.pose.position, goal.target_pose.pose.orientation)


        while not success and not rospy.is_shutdown():
            try:
                vel_msg = self.CalcLinearAndAngular(target_pose)
                print vel_msg
                self.vel_pub.publish(vel_msg)
                if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
                    success = True
                r.sleep()
            except KeyboardInterrupt:
                print('interrupted!')

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def CalcLinearAndAngular(self, target_pose, max_vel = 0.1, error_threshold = 0.01):

        angl_vel_mod = 6
        lin_vel_mod = 4


        now = time.time()
        trans = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
        # print now, trans.header.stamp.secs,  trans.header.stamp.nsecs/1000.
        x = target_pose.translation.x - trans.transform.translation.x
        y = target_pose.translation.y - trans.transform.translation.y

        vel = [x, y, 0., 0.]

        q_cur = trans.transform.rotation
        q_cur= [q_cur.x, q_cur.y, q_cur.z, q_cur.w]
        q_cur_inv = [q_cur[0], q_cur[1], q_cur[2], -q_cur[3]]


        result = quaternion_multiply(quaternion_multiply(q_cur_inv, vel),quaternion_conjugate(q_cur_inv))


        q_traget = target_pose.rotation
        q_traget = [q_traget.x, q_traget.y, q_traget.z, q_traget.w]

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


if __name__ == '__main__':
    rospy.init_node('MyBaseController')
    try:
        server = SimulationBaseController(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutdown simulator demo"
