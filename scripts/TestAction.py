#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
import actionlib_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import numpy as np

def test_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/whole_body_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print "waiting finished"
    # # Creates a goal to send to the action server.
    goal_temp = control_msgs.msg.FollowJointTrajectoryGoal()
    goal_temp.trajectory.joint_names = ["r_shoulder_pan_joint"]

    # point = trajectory_msgs.msg.JointTrajectoryPoint([-0.1],[0.0],[0.0], [0.0], rospy.Duration(1.0))
    goal_temp.trajectory.points = [trajectory_msgs.msg.JointTrajectoryPoint([-i],[0.0],[0.0], [0.0], rospy.Duration(0.1)) for i in np.linspace(0.0, 0.8, 80)]
    # goal_temp.trajectory.points = [point]
    # goal_temp.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint(-0.1,0.0,0.0, 0.0, 0.0))


    # Sends the goal to the action server.
    client.send_goal(goal_temp)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    print "success"
    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_client_py')
        result = test_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
