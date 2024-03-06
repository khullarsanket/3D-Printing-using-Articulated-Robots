#!/usr/bin/env python
from ikpy import chain
import numpy as np
import rospy
import actionlib
import time
import math
import sys

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

my_chain = chain.Chain.from_urdf_file("/home/sk/yaskawa_ws/src/motoman/motoman_gp50_support/urdf/gp50.urdf")
def main():
    rospy.init_node('motion  _node')

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
        # trajectory_msgs.msg._JointTrajectoryPoint.JointTrajectoryPoint
        rospy.logfatal("Mismatch between joints specified and seen in current "
            "JointState. Expected: '{}', got: '{}'. Cannot continue.".format(
                ', '.join(robot_joint_states.name),
                ', '.join(goal.trajectory.joint_names)))
        sys.exit(1)

    qi = robot_joint_states.position	#current trajectory point
    qdot = [0.0] * len(goal.trajectory.joint_names)
    goal.trajectory.points.append(JointTrajectoryPoint(positions=qi,
            velocities=qdot, time_from_start=rospy.Duration(0)))
    
    # q_list = []
    # q_list.append(qi)
    for c in range(0,210): # Trajectory for point A to point B
        target_position = np.matrix([[1.345-0.000587*c,0.0001989*c,1.620-.0062*c]])
        array = np.eye(4)
        array[0,3] = target_position[0,0]
        array[1,3] = target_position[0,1]
        array[2,3] = target_position[0,2]

        target = array

        angles = my_chain.inverse_kinematics(target)

        # if angles[3] > math.pi/2:

        q1 = [angles[1],angles[2],angles[3],0,angles[2]-angles[3]-(math.pi/2),math.pi/2]
        # q_list.append(q1)
        # temp_array = np.array(q_list)
        # else:
            # joint_angles = [angles[1],angles[2],angles[3],angles[4],angles[3]-angles[2]-(math.pi/2),-math.pi/2]
        # qdot = ((temp_array[c+1]-temp_array[c])/0.1).tolist()
        rospy.loginfo(q1)
        # rospy.loginfo(qdot)
        # print(goal.trajectory)
        # print(JointTrajectoryPoint)
        # exit()
        goal.trajectory.points.append(JointTrajectoryPoint(positions=q1,
            velocities=qdot, time_from_start=rospy.Duration(0.1+0.1*c)))
       
        # q1= [0, 0.0, 0.0, 0.0, -math.pi/2, math.pi/2]		#trajectory point to be reached

    # q2= [0, 0.5, 0, 1.57, 0.2, 1.57]

    x_range = np.linspace(1.345-0.000587*c,1.5,10)
    # x_list = []
    # x_list.append(q_list[-1])
    for i,x in enumerate(x_range):
        target_position = np.matrix([[x,0.0001989*c,1.620-.0062*c]])
        array = np.eye(4)
        array[0,3] = target_position[0,0]
        array[1,3] = target_position[0,1]
        array[2,3] = target_position[0,2]

        target = array

        angles = my_chain.inverse_kinematics(target)

        q1 = [angles[1],angles[2],angles[3],0,angles[2]-angles[3]-(math.pi/2),math.pi/2]
        rospy.loginfo(q1)
        # x_list.append(q1)
        # temp_array_3 = np.array(x_list)

        # qdot = ((temp_array_3[i+1]-temp_array_3[i])/0.1).tolist()
        goal.trajectory.points.append(JointTrajectoryPoint(positions=q1,
        velocities=qdot, time_from_start=rospy.Duration(21.1+0.1*i)))


    # y_range =  np.linspace(0.0001989*c,-0.5,20)
    # for i,y in enumerate(y_range):
    #     target_position = np.matrix([[1.5,y,1.620-.0062*c]])
    #     array = np.eye(4)
    #     array[0,3] = target_position[0,0]
    #     array[1,3] = target_position[0,1]
    #     array[2,3] = target_position[0,2]

    #     target = array

    #     angles = my_chain.inverse_kinematics(target)

    #     q1 = [angles[1],angles[2],angles[3],0,angles[2]-angles[3]-(math.pi/2),math.pi/2]
        
    #     goal.trajectory.points.append(JointTrajectoryPoint(positions=q1,
    #     velocities=qdot, time_from_start=rospy.Duration(22.2+0.1*i)))
    # theta_range  = np.linspace(0,-math.pi/4,100)

    #     # for d in range(250,502): 
    # q2_list = []
    # q2_list.append(x_list[-1])
    # for i,theta in enumerate(theta_range): 

    #         # target_position = np.matrix([[ 0.900+ math.cos(d/100)*.400, math.sin(d/100)*.600,.0136+0.00002*d]])
    #         # # target_position = np.matrix([[ 0.900+ math.cos(d/100)*.400, math.sin(d/100)*.600,.0136]])
    #         # target_position = np.matrix([[ 0.100,0+0.01*d,.0136]])
    #     target_position = np.matrix([[ 1.5*math.cos(theta),0.0001989*c+1.5*math.sin(theta),1.620-.0062*c]])
    #     array = np.eye(4)
    #     array[0,3] = target_position[0,0]
    #     array[1,3] = target_position[0,1]
    #     array[2,3] = target_position[0,2]

    #     target = array

    #     angles = my_chain.inverse_kinematics(target)

    #     # Make the robot come to a complete stop at each trajectory point (ie:
    #     # zero target velocity).
    #     q2 = [angles[1],angles[2],angles[3],0,angles[2]-angles[3]-(math.pi/2),math.pi/2]

    #     q2_list.append(q2)
    #     temp_array_2 = np.array(q2_list)

    #     qdot = ((temp_array_2[i+1]-temp_array_2[i])/0.1).tolist()

        
    #     # add points to the trajectory
    #     # rospy.loginfo(q2)
    #     rospy.loginfo(qdot)

    #     goal.trajectory.points.append(JointTrajectoryPoint(positions=q2,
    #         velocities=qdot, time_from_start=rospy.Duration(23+0.1*i)))
    
        # goal.trajectory.points.append(JointTrajectoryPoint(positions=q1,
        #   velocities=qdot, time_from_start=rospy.Duration(10.0)))



    qf= [0, 0, 0, 0, -1.571,1.571]
    qdot = [0.0] * len(goal.trajectory.joint_names)
    goal.trajectory.points.append(JointTrajectoryPoint(positions=qf,
    velocities=qdot, time_from_start=rospy.Duration(36)))

    # goal constructed, submit it for execution
    rospy.loginfo("Submitting goal ..")
    client.send_goal(goal)
    rospy.loginfo("Waiting for completion ..")
    client.wait_for_result()
    rospy.loginfo('Done.')

if __name__ == '__main__':
    main()
