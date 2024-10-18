#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController

if __name__ == "__main__":
  rospy.init_node("init_telepresence")
  rospy.loginfo("Waiting for play_motion...")
  client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  rospy.wait_for_message("/joint_states", JointState)
  rospy.sleep(3.0)

  rospy.loginfo("Tuck arm...")
  goal = PlayMotionGoal()
  goal.motion_name = 'home'
  goal.skip_planning = False

  client.send_goal(goal)
  client.wait_for_result(rospy.Duration(10.0))
  rospy.loginfo("Arm tucked.")

  rospy.wait_for_service("/controller_manager/switch_controller")
  manager = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

  start_controllers=["arm_tp_controller", "gripper_tp_controller", "torso_tp_controller", "head_tp_controller"]
  stop_controllers=["arm_controller", "gripper_controller", "torso_controller", "head_controller"]

  rospy.loginfo("Switching controllers...")
  response = manager(start_controllers=start_controllers,
                     stop_controllers=stop_controllers,
                     strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers")

  rospy.loginfo("...done.")
