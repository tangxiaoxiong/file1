#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs
import tf

from moveit_commander import MoveGroupCommander
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from copy import deepcopy

pose_bias=[0,0.00,0]
def callback(pose):
	object_position_info = pose.position
	object_orientation_info = pose.orientation
	print object_position_info

	moveit_commander.roscpp_initialize(sys.argv)
	#rospy.init_node('move_group_grasp', anonymous=True)
	robot = moveit_commander.robot.RobotCommander()
	arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
	hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
	print("Point Home")
	arm_group.set_named_target('home_j')
	arm_group.go()
	hand_group.set_named_target('open')
	hand_group.go()
	pose_target = arm_group.get_current_pose().pose
	# Block point top

	pose_target.position.x = object_position_info.x+pose_bias[0]
	pose_target.position.y = object_position_info.y+pose_bias[1]
	pose_target.position.z = object_position_info.z+0.3
	arm_group.set_pose_target(pose_target)
	arm_group.go()
	print("Point Catch")
	#rospy.sleep(1)	
	# Block point1
	pose_target.position.z = pose_target.position.z-0.18

	arm_group.set_pose_target(pose_target)
	arm_group.go()
	print("Point Lift")
	#rospy.sleep(1)

	hand_group.set_named_target("close")
	plan = hand_group.go()
	rospy.sleep(1)

	pose_target.position.z = object_position_info.z+0.35
	arm_group.set_pose_target(pose_target)
	arm_group.go()
	print("Point Drop")
	rospy.sleep(1)

	pose_target.position.x = 0.33
	pose_target.position.y = -0.33
	pose_target.position.z = object_position_info.z+0.35
	arm_group.set_pose_target(pose_target)
	arm_group.go()
	rospy.sleep(1)
	hand_group.set_named_target("open")
	plan = hand_group.go()
	   
	moveit_commander.roscpp_shutdown()

def object_position_sub():
    rospy.Subscriber("/objection_position_pose",Pose,callback,queue_size=10)

if __name__ == "__main__":
    rospy.init_node('object_position_sub_And_grasp_node',anonymous=True)
    object_position_sub()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
                
        rate.sleep()
