import rospy
from giskardpy.python_interface import GiskardWrapper


def joint_space_client():
    angle = 0.0
    giskard = GiskardWrapper()
    rospy.sleep(0.5)
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveAction) to the constructor.

    # Creates a goal to send to the action server.
    # Waits until the action server has started up and started
    # listening for goals.
    angle += 0.8
    rospy.loginfo("%f", angle)

    gaya_pose = {'r_shoulder_pan_joint': -1.7125,
                 'r_shoulder_lift_joint': -0.25672,
                 'r_upper_arm_roll_joint': -1.46335,
                 'r_elbow_flex_joint': -2.12216,
                 'r_forearm_roll_joint': 1.76632,
                 'r_wrist_flex_joint': -0.10001,
                 'r_wrist_roll_joint': 0.05106,

                 'l_shoulder_pan_joint': 1.9652,
                 'l_shoulder_lift_joint': - 0.26499,
                 'l_upper_arm_roll_joint': 1.3837,
                 'l_elbow_flex_joint': - 2.1224,
                 'l_forearm_roll_joint': 16.99,
                 'l_wrist_flex_joint': - 0.10001,
                 'l_wrist_roll_joint': 0}
    giskard.allow_all_collisions()
    giskard.set_joint_goal(gaya_pose)
    giskard.plan_and_execute()

    return giskard.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_space_client')
        result = joint_space_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
