import logging
import os
import rospy

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk # python3

try:
    from zed_recorder.srv import ZedRecord, ZedRecordRequest
    AVAILABLE = True
except ImportError:
    AVAILABLE = False

logger = logging.getLogger('ada_assistance_policy')

ZED_RECORDER_CONFIG_NAME = 'zed_recorder'

class RemoteRecorder:
    def __init__(self, log_dir, topic):
        if not AVAILABLE:
            raise RuntimeError('ZED message files not found.')

        self.filename = os.path.join(log_dir, 'user_video.svo')
        try:
            rospy.wait_for_service(config[ZED_RECORDER_CONFIG_NAME]['topic'], timeout=1.)
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


class RemoteRecorderConfigFrame(tk.Frame, object):
    def __init__(self, parent, initial_config):
        super(RemoteRecorderConfigFrame, self).__init__(parent)
        initial_config = initial_config.get(ZED_RECORDER_CONFIG_NAME, {})

        self.enabled_var = tk.BooleanVar(value=AVAILABLE and initial_config.get('enabled', False))
        self.enabled_checkbox = tk.Checkbutton(self, variable=self.enabled_var,
                state=tk.NORMAL if AVAILABLE else tk.DISABLED)
        self.enabled_label = tk.Label(self, text="Enable ZED recording")
        self.enabled_checkbox.grid(row=0, column=0, sticky=tk.N+tk.S+tk.E)
        self.enabled_label.grid(row=0, column=1, sticky=tk.N+tk.S+tk.E+tk.W)


        self.topic_var = tk.StringVar(value=initial_config.get('topic', '/zed_remote_recorder'))
        self.topic_entry = tk.Entry(self, textvariable=self.topic_var, state=tk.NORMAL if AVAILABLE else tk.DISABLED)
        self.topic_label = tk.Label(self, text='Topic for ZED remote recorder service')
        self.topic_entry.grid(row=1, column=0, sticky=tk.N+tk.S+tk.E+tk.W)
        self.topic_label.grid(row=1, column=1, sticky=tk.N+tk.S+tk.W)

    def get_config(self):
        return { ZED_RECORDER_CONFIG_NAME: {
            'enabled': self.enabled_var.get(),
            'topic': self.topic_var.get()
        }}

    def set_state(self, state):
        if AVAILABLE: # if it's not available, we never want to enable it
            self.enabled_checkbox.configure(state=state)
            self.topic_entry.configure(state=state)
                
def get_zed_remote_recorder(log_dir, config):
    if ZED_RECORDER_CONFIG_NAME in config and config[ZED_RECORDER_CONFIG_NAME]['enabled']:
        return RemoteRecorder(log_dir, config[ZED_RECORDER_CONFIG_NAME]['topic'])
    else:
        return None