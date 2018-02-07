import rospy
from bypassable_action import BypassableAction, ActionException
from trajectory_actions import LookAtFace, LookAtPlate, Serve
from detect_morsal import DetectMorsal
from get_morsal import GetMorsal
from std_msgs.msg import String
import time


from ada_teleoperation.DataRecordingUtils import *

import logging
logger = logging.getLogger('ada_meal_scenario')

from direct_teleop_action import DirectTeleopAction

class BiteServing(BypassableAction):

    def __init__(self, bypass = False):
        BypassableAction.__init__(self, 'BiteServing', bypass=bypass)

    def execute(self, manip, env, method, ui_device, state_pub, 
                detection_sim=False, record_trial=False, file_directory=None, transition_function=lambda x,y: x+y):
        
        # Move to look at face
        #action = LookAtFace(bypass = self.bypass)
        #state_pub.publish(action.name)
        #action.execute(manip)

        if record_trial:
          if file_directory is None:
            file_directory = rospkg.RosPack().get_path('ada_meal_scenario') + '/trajectory_data'

          rosbag_topic_names = ['/ada_tasks', '/ada/joy', '/perception/morsel_detection', '/joint_states', '/myo_raw/myo_arm', '/myo_raw/myo_emg', '/myo_raw/myo_imu', '/myo_raw/myo_ori', '/myo_raw/myo_vibrate']
          filename_trajdata, filename_bag = get_next_filename_pair(file_directory=file_directory)

          rosbag_process = start_rosbag(rosbag_topic_names, filename=filename_bag)
          state_pub.publish("recording data to " + str(filename_bag))
        else:
          filename_trajdata = None

#        #if direct teleop, skip sequence
#        if method == 'direct':
#          state_pub.publish("Direct Teleop")
#          direct_teleop_action = DirectTeleopAction(bypass=self.bypass)
#          direct_teleop_action.execute(manip, ui_device, filename_trajdata=filename_trajdata)
#
#          #make sure we end at serving
#          manip.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration', execute=True)
#        else:


        try: 
          # Move to look at plate
          action = LookAtPlate(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip)

          # Detect morsal
          if self.bypass:
              detection_sim = True
          action = DetectMorsal(bypass = detection_sim)
          state_pub.publish(action.name)
          action.execute(manip.GetRobot())
                      
          # Move to get object
          action = GetMorsal(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip, method, ui_device, state_pub, 
                         filename_trajdata=filename_trajdata, transition_function=transition_function)

    
          # Serve the morsal
          action = Serve(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip)

          state_pub.publish("Finished bite serving")
          if record_trial:
            stop_rosbag(rosbag_process)

        except ActionException, e:
          state_pub.publish("Failed to run bite serving")
          if record_trial:
            stop_rosbag(rosbag_process)
          raise

