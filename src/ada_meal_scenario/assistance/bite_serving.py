from bypassable_action import BypassableAction, ActionException
from trajectory_actions import LookAtPlate, Serve
from detect_morsel import DetectMorsel
from get_morsel import GetMorsel
from direct_teleop_action import DirectTeleopAction
from std_msgs.msg import String
import os.path


from ada_teleoperation.DataRecordingUtils import *

import logging
logger = logging.getLogger('ada_meal_scenario')


try:
    from zed_recorder.srv import ZedRecord, ZedRecordRequest
except ImportError:
    pass

class RemoteRecorder:
    def __init__(self, topic, filename):
        self.filename = filename + '.svo'
        try:
            rospy.wait_for_service(topic, timeout=1.)
            self.service = rospy.ServiceProxy(topic, ZedRecord)
        except Exception as ex:
            logger.warn('Failed to connect to remote ZED recorder!')
            self.service = None
         
    def start(self):
        logger.info('Starting remote ZED recorder to {}'.format(self.filename))
        if self.service is not None:
            try:
                res = self.service(command=ZedRecordRequest.START, filename=self.filename, fps=0)
                if not res.ok:
                    logger.warn('Failed to start remote ZED recorder: {}'.format(res.message))
            except Exception as ex:
                logger.warn('Exception when starting remote ZED recorder: {}'.format(str(e)))

    def stop(self):
        if self.service is not None:
            try:
                res = self.service(command=ZedRecordRequest.STOP)
                if not res.ok:
                    logger.warn('Failed to stop remote ZED recorder: {}'.format(res.message))
            except Exception as ex:
                logger.warn('Exception when stopping remote ZED recorder: {}'.format(str(e)))
                

class BiteServing(BypassableAction):

    def __init__(self, bypass = False):
        BypassableAction.__init__(self, 'BiteServing', bypass=bypass)

    def execute(self, manip, env, method, ui_device, state_pub, 
                detection_sim=False, record_trial=False, file_directory=None, transition_function=lambda x,y: x+y,prediction_option= "Goal"):
        
        if record_trial:
          if file_directory is None:
            file_directory = rospkg.RosPack().get_path('ada_meal_scenario') + '/trajectory_data'

          rosbag_topic_names = ['/ada_tasks', '/ada/joy', '/perception/morsel_detection', '/joint_states', '/myo_raw/myo_arm', '/myo_raw/myo_emg', '/myo_raw/myo_imu', '/myo_raw/myo_ori', '/myo_raw/myo_vibrate']
          filename_trajdata, filename_bag = get_next_filename_pair(file_directory=file_directory)

          rosbag_process = start_rosbag(rosbag_topic_names, filename=filename_bag)
          state_pub.publish("recording data to " + str(filename_bag))
          
          remote_recorder = RemoteRecorder('/zed_recorder', 
                                           os.path.splitext(os.path.relpath(filename_trajdata, os.path.join(file_directory, '..')))[0].replace('/', '_'))
          remote_recorder.start()
        else:
          filename_trajdata = None


        try: 
          # Move to look at plate
          action = LookAtPlate(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip)

          # Detect morsel
          if self.bypass:
              detection_sim = True
          action = DetectMorsel(bypass = detection_sim)
          state_pub.publish(action.name)
          action.execute(manip.GetRobot())
                      
          # Move to get object
          action = GetMorsel(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip, method, ui_device, state_pub, 
                         filename_trajdata=filename_trajdata, transition_function=transition_function,prediction_option= predict_option)

          # Serve the morsel
          action = Serve(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip)

          state_pub.publish("Finished bite serving")
          if record_trial:
            stop_rosbag(rosbag_process)
            remote_recorder.stop()

        except ActionException, e:
          state_pub.publish("Failed to run bite serving")
          if record_trial:
            stop_rosbag(rosbag_process)
            remote_recorder.stop()
          raise

