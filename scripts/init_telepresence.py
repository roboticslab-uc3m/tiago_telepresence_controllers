#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers, SwitchController

if __name__ == "__main__":
  rospy.init_node("init_telepresence")

  rospy.wait_for_message("/joint_states", JointState)
  rospy.sleep(3.0)

  rospy.wait_for_service("/controller_manager/list_controllers")
  rospy.wait_for_service("/controller_manager/switch_controller")

  manager_list = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
  manager_switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

  cs = manager_list().controller
  tp_controllers = [c.name for c in cs if c.name.endswith("_tp_controller")]
  start_controllers = [c.name for c in cs if c.name in tp_controllers and c.state == "stopped"]
  stop_controllers = [c.name for c in cs if c.name.replace("_controller", "_tp_controller") in tp_controllers and c.state == "running"]

  rospy.loginfo("Switching controllers...")
  response = manager_switch(start_controllers=start_controllers,
                            stop_controllers=stop_controllers,
                            strictness=2)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers")

  rospy.loginfo("...done.")
