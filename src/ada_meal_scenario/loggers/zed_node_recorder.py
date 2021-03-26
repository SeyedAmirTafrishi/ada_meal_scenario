import logging
import os
import rospy

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk # python3

try:
    from zed_interfaces.srv import start_svo_recording, stop_svo_recording
    AVAILABLE = True
except ImportError:
    AVAILABLE = False

logger = logging.getLogger('ada_assistance_policy')

ZED_NODE_RECORDER_CONFIG_NAME = 'zed_node'

class ZedNodeRecorder:
    def __init__(self, log_dir, topic):
        if not AVAILABLE:
            raise RuntimeError('ZED node connection not found.')

        self.filename = os.path.join(log_dir, 'user_video.svo')
        try:
            rospy.wait_for_service(topic + '/start_svo_recording', timeout=1.)
            self.start_service = rospy.ServiceProxy(topic + '/start_svo_recording', start_svo_recording)
            self.stop_service = rospy.ServiceProxy(topic + '/stop_svo_recording', stop_svo_recording)
        except Exception as ex:
            logger.warn('Failed to connect to ZED node!')
            self.start_service = None
            self.stop_service = None
         
    def start(self):
        logger.info('Starting recording for ZED node to {}'.format(self.filename))
        if self.start_service is not None:
            try:
                res = self.start_service(svo_filename=self.filename)
                if res.result:
                    logger.info('Started recording on ZED node: {}'.format(res.info))
                else:
                    logger.warn('Failed to start recording on ZED node: {}'.format(res.info))
            except Exception as ex:
                logger.warn('Exception when starting recording on ZED node: {}'.format(str(ex)))
        else:
            logger.warn('Attempted to start zed recording but service was not created')

    def stop(self):
        if self.stop_service is not None:
            try:
                res = self.stop_service()
                if res.done:
                    logger.info('Stopped recording on ZED node: {}'.format(res.info))
                else:
                    logger.warn('Failed to stop recording on ZED node: {}'.format(res.info))
            except Exception as ex:
                logger.warn('Exception when stopping recording on ZED node: {}'.format(str(e)))


class ZedNodeRecorderConfigFrame(tk.LabelFrame, object):
    def __init__(self, parent, initial_config):
        super(ZedNodeRecorderConfigFrame, self).__init__(parent, text='ZED Node')
        initial_config = initial_config.get(ZED_NODE_RECORDER_CONFIG_NAME, {})

        self.enabled_var = tk.BooleanVar(value=AVAILABLE and initial_config.get('enabled', False))
        self.enabled_checkbox = tk.Checkbutton(self, variable=self.enabled_var,
                state=tk.NORMAL if AVAILABLE else tk.DISABLED, text="Enable ZED node recording")
        self.enabled_checkbox.grid(row=0, column=0, columnspan=2, sticky=tk.N+tk.W)

        self.topic_var = tk.StringVar(value=initial_config.get('topic', '/zed'))
        self.topic_entry = tk.Entry(self, textvariable=self.topic_var, state=tk.NORMAL if AVAILABLE else tk.DISABLED)
        self.topic_label = tk.Label(self, text='Parent topic')
        self.topic_label.grid(row=1, column=0, sticky=tk.N+tk.S+tk.W)
        self.topic_entry.grid(row=1, column=1, sticky=tk.N+tk.S+tk.E+tk.W)

        self.columnconfigure(1, weight=1)

    def get_config(self):
        return { ZED_NODE_RECORDER_CONFIG_NAME: {
            'enabled': self.enabled_var.get(),
            'topic': self.topic_var.get()
        }}

    def set_state(self, state):
        if AVAILABLE: # if it's not available, we never want to enable it
            self.enabled_checkbox.configure(state=state)
            self.topic_entry.configure(state=state)
                
def get_zed_node_recorder(log_dir, config):
    if log_dir is not None and ZED_NODE_RECORDER_CONFIG_NAME in config and config[ZED_NODE_RECORDER_CONFIG_NAME]['enabled']:
        return ZedNodeRecorder(log_dir, config[ZED_NODE_RECORDER_CONFIG_NAME]['topic'])
    else:
        return None