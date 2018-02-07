#creates a shared autonomy policy that runs essentially as an action

import logging, numpy, prpy, os
from ada_assistance_policy.AdaHandler import AdaHandler
from ada_assistance_policy.Goal import Goal
import rospy
import rospkg

from bypassable_action import ActionException, BypassableAction

from ada_teleoperation.DataRecordingUtils import TrajectoryData

#import prpy.rave, prpy.util
#from prpy.planning.base import PlanningError
#from prpy.exceptions import TrajectoryNotExecutable
#import time

class AssistancePolicyAction(BypassableAction):
    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'ASSISTANCE_POLICY', bypass=bypass)
        
    def _run(self, manip, objects, desired_ee_poses, ui_device, 
             fix_magnitude_user_command=False, blend_only=False, filename_trajdata=None, transition_function=lambda x,y: x+y):
        robot = manip.GetRobot()
        env = robot.GetEnv()

        if filename_trajdata:
          traj_data_recording = TrajectoryData(filename_trajdata)
        else:
          traj_data_recording = None

        all_goals = [Goal(obj.GetTransform(), [desired_ee_pose]) for obj, desired_ee_pose in zip(objects, desired_ee_poses)]
        ada_handler = AdaHandler(env, robot, all_goals, objects, input_interface_name=ui_device, num_input_dofs=2, use_finger_mode=False)
        ada_handler.execute_policy(simulate_user=False, 
                                   blend_only=blend_only, 
                                   fix_magnitude_user_command=fix_magnitude_user_command, 
                                   traj_data_recording=traj_data_recording, 
                                   transition_function=transition_function)
        #ada_handler.execute_direct_teleop(simulate_user=False)



