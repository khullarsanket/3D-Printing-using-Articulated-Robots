#!/usr/bin/env python


import rospy
import actionlib
import time
import math
import sys

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    rospy.init_node('home_node')

    # create client and make sure it's available
    client = actionlib.SimpleActionClient('joint_trajectory_action', FollowJointTrajectoryAction)
    rospy.loginfo('Waiting for driver\'s action server to become available ..')
    client.wait_for_server()
    rospy.loginfo('Connected to trajectory action server')

    # setup simple goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names =  ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'] #check that these match with the ones in the robot_joint_states

    # motoman_driver only accepts goals with trajectories that start at the
    # current robot state, so retrieve that and use it as the first point in
    # the trajectory
    robot_joint_states = rospy.wait_for_message('/joint_states', JointState)

    # make sure the state we get contains the same joints (both amount and names)
    if set(goal.trajectory.joint_names) != set(robot_joint_states.name):
        rospy.logfatal("Mismatch between joints specified and seen in current "
            "JointState. Expected: '{}', got: '{}'. Cannot continue.".format(
                ', '.join(robot_joint_states.name),
                ', '.join(goal.trajectory.joint_names)))
        sys.exit(1)

    q0 = robot_joint_states.position	#current trajectory point

    q1= [0, 0, 0, 0, -1.571,1.571]		#trajectory point to be reached

    q2= [0, 0, 0, 0, -1.571,1.571]

    

    # Make the robot come to a complete stop at each trajectory point (ie:
    # zero target velocity).
    qdot = [0.0] * len(goal.trajectory.joint_names)

    # add points to the trajectory

    goal.trajectory.points.append(JointTrajectoryPoint(positions=q0,
        velocities=qdot, time_from_start=rospy.Duration(0)))

    goal.trajectory.points.append(JointTrajectoryPoint(positions=q1,
        velocities=qdot, time_from_start=rospy.Duration(5.0)))
    
    goal.trajectory.points.append(JointTrajectoryPoint(positions=q2,
        velocities=qdot, time_from_start=rospy.Duration(10.0)))

    # goal constructed, submit it for execution
    rospy.loginfo("Submitting goal ..")
    client.send_goal(goal)
    rospy.loginfo("Waiting for completion ..")
    client.wait_for_result()
    rospy.loginfo('Done.')


if __name__ == '__main__':
    main()
