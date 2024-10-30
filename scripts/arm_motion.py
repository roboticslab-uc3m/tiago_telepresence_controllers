#!/usr/bin/env python

import argparse
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from tiago_telepresence_controllers.srv import ArmMotion, ArmMotionResponse

parser = argparse.ArgumentParser()
parser.add_argument("--arm", type=str, default=None)

current_state = None
list_controllers = None
tp_controllers = []

def pre_switch_controllers():
  cs = list_controllers().controller
  start_controllers = [c.name for c in cs if c.name.replace("_controller", "_tp_controller") in tp_controllers and c.state == "stopped"]
  stop_controllers = [c.name for c in cs if c.name in tp_controllers and c.state == "running"]

  rospy.loginfo("Switching controllers (pre)...")
  rospy.loginfo("Start: %s" % start_controllers)
  rospy.loginfo("Stop: %s" % stop_controllers)

  response = switch_controller(start_controllers=start_controllers,
                               stop_controllers=stop_controllers,
                               strictness=SwitchControllerRequest.STRICT)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers (pre)")

def post_switch_controllers():
  cs = list_controllers().controller
  start_controllers = [c.name for c in cs if c.name in tp_controllers and c.state == "stopped"]
  stop_controllers = [c.name for c in cs if c.name.replace("_controller", "_tp_controller") in tp_controllers and c.state == "running"]

  rospy.loginfo("Switching controllers (post)...")
  rospy.loginfo("Start: %s" % start_controllers)
  rospy.loginfo("Stop: %s" % stop_controllers)

  response = switch_controller(start_controllers=start_controllers,
                               stop_controllers=stop_controllers,
                               strictness=SwitchControllerRequest.STRICT)

  if not response.ok:
    rospy.logfatal("Failed to switch controllers (post)")

def handle_arm_motion(req):
  global current_state

  if req.command == current_state:
    rospy.loginfo("Motion '%s' already completed." % req.command)
    return ArmMotionResponse(success=True)

  if len(tp_controllers) == 0:
    cs = list_controllers().controller
    tp_controllers.extend([c.name for c in cs if c.name.endswith("_tp_controller")])

  rospy.loginfo("Executing motion: %s", req.command)

  goal = PlayMotionGoal()
  goal.motion_name = req.command
  goal.skip_planning = False

  pre_switch_controllers()
  rospy.sleep(1.0) # important

  client.send_goal(goal)
  client.wait_for_result(rospy.Duration(10.0))
  action_res = client.get_result()

  res = ArmMotionResponse()

  # for some reason, action_res is sometimes None
  if action_res is None or action_res.error_code == PlayMotionResult.SUCCEEDED:
    rospy.loginfo("Motion '%s' completed." % req.command)
    res.success = True
    current_state = req.command
  else:
    rospy.logerr(action_res.error_string)
    res.success = False
    res.message = action_res.error_string

  post_switch_controllers()
  return res

if __name__ == "__main__":
  rospy.init_node("tp_arm_motion")

  args, unknown = parser.parse_known_args()

  if args.arm is not None: # TIAGo++
    if args.arm not in ["left", "right"]:
      rospy.logfatal("Invalid argument for --arm: %s" % args.arm)
      exit(1)

    current_state = "home_" + args.arm
  else: # TIAGo
    current_state = "home"

  rospy.loginfo("Waiting for play_motion...")

  client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  controllers = rospy.get_param("/play_motion/controllers")

  rospy.wait_for_service("/controller_manager/list_controllers")
  rospy.wait_for_service("/controller_manager/switch_controller")

  list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
  switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

  s = rospy.Service("/tp_arm_motion/command", ArmMotion, handle_arm_motion)

  rospy.spin()
