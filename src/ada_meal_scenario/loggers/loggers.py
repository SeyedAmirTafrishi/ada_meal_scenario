
import copy
import os
try:
    import Tkinter as tk
    import tkFileDialog as tkfile
    import tkFont as tkfont
except ImportError:  # python3
    import tkinter as tk
    import tkfiledialog as tkfile
    import tkfont
import yaml

import rospkg

from ada_teleoperation import DataRecordingUtils
from ada_meal_scenario.loggers.pupil_recorder import PupilRecorderConfigFrame, get_pupil_recorder
from ada_meal_scenario.loggers.rosbag_recorder import RosbagRecorderConfigFrame, get_rosbag_recorder
from ada_meal_scenario.loggers.zed_remote_recorder import RemoteRecorderConfigFrame, get_zed_remote_recorder

LOGGING_CONFIG_NAME = 'logging'

class LoggingOptions(tk.Frame, object):
    __USER_ID_INVALID_BG__ = "#cc0000"
    def __init__(self, parent, initial_config={}):
        super(LoggingOptions, self).__init__(parent)
        initial_config = initial_config.get(LOGGING_CONFIG_NAME, {})

        label_font = tkfont.nametofont("TkDefaultFont").copy()
        label_font.configure(weight='bold')

        self.label = tk.Label(
            self, text="Logging Options", font=label_font)
        self.label.grid(row=0, sticky=tk.E+tk.W)


        self.logging_frame = tk.Frame(self)
        base_dir = initial_config.get('base_dir', 
                os.path.join(rospkg.RosPack().get_path('ada_meal_scenario'), 'trajectory_data'))
        
        # choose top dir for logging
        self.data_root_var = tk.StringVar(value=base_dir)
        self.data_root_label = tk.Label(
            self.logging_frame, textvariable=self.data_root_var, wraplength=200, justify=tk.LEFT)
        self.data_root_label.grid(row=0, column=0, sticky=tk.W)
        self.data_root_button = tk.Button(self.logging_frame, text='Select data directory', command=self._set_data_root)
        self.data_root_button.grid(row=0, column=1, sticky=tk.W)

        # choose user id
        self.user_id_var = tk.StringVar()
        self.update_next_user_id()
        self.user_id_var.trace("w", self._validate_user_id)
        self.user_id_entry = tk.Entry(
            self.logging_frame, textvariable=self.user_id_var)
        self.user_id_entry.grid(row=2, column=0, sticky=tk.N+tk.E+tk.W)
        self.user_id_label = tk.Label(self.logging_frame, text='User ID')
        self.user_id_label.grid(row=2, column=1, sticky=tk.N+tk.W)

        self.logging_frame.grid(row=1, column=0, sticky=tk.N+tk.S+tk.E+tk.W)

        self.user_id_orig_bg = self.user_id_entry.cget("bg")

        # additional logging options
        self.pupil_config = PupilRecorderConfigFrame(self, initial_config)
        self.pupil_config.grid(row=2, column=0, sticky=tk.N+tk.E+tk.W+tk.S)
        self.zed_config = RemoteRecorderConfigFrame(self, initial_config)
        self.zed_config.grid(row=2, column=1, sticky=tk.N+tk.E+tk.W+tk.S)
        self.rosbag_config = RosbagRecorderConfigFrame(self, initial_config)
        self.rosbag_config.grid(row=1, column=1, sticky=tk.N+tk.E+tk.W+tk.S)

    def _set_data_root(self):
        data_root = tkfile.askdirectory(initialdir=self.data_root_var.get(), title='Choose root directory for logging')
        if data_root is not None:
            self.data_root_var.set(data_root)

    def _get_data_dir(self):
        return DataRecordingUtils.get_filename(
            self.data_root_var.get(), DataRecordingUtils.user_folder_base_default, self.user_id_var.get(), '')
    
    def _validate_user_id(self, *_):
        data_dir = self._get_data_dir()
        if os.path.exists(data_dir):
            self.user_id_entry.config(bg=LoggingOptions.__USER_ID_INVALID_BG__)
        else:
            self.user_id_entry.config(bg=self.user_id_orig_bg)

    def get_config(self):
        data_dir = self._get_data_dir()
        if os.path.exists(data_dir):
            raise ValueError("Directory exists: {}".format(data_dir))
        base_res = {
            'base_dir': self.data_root_var.get(),
            'data_dir': data_dir
        }
        base_res.update(self.pupil_config.get_config())
        base_res.update(self.zed_config.get_config())
        base_res.update(self.rosbag_config.get_config())
        return { LOGGING_CONFIG_NAME: base_res }

    def set_state(self, state):
        self.data_root_button.configure(state=state)
        self.user_id_entry.configure(state=state)
        self.pupil_config.set_state(state=state)
        self.zed_config.set_state(state=state)
        self.rosbag_config.set_state(state=state)

    def update_next_user_id(self):
        # when we've finished a trial, we need to advance the user id
        default_user_id, _ = DataRecordingUtils.get_next_available_user_ind(
            self.data_root_var.get(), make_dir=False)
        self.user_id_var.set(default_user_id)

def get_log_dir(config):
    return config.get(LOGGING_CONFIG_NAME, {}).get('data_dir', None)

def get_loggers(config):
    if LOGGING_CONFIG_NAME not in config:
        return []

    config = config[LOGGING_CONFIG_NAME]
    log_dir = config.get('data_dir', None)
    if log_dir is not None:
        # make sure the directory exists
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)

        loggers = []

        zed_logger = get_zed_remote_recorder(log_dir, config)
        if zed_logger is not None:
            loggers.append(zed_logger)

        pupil_logger = get_pupil_recorder(log_dir, config)
        if pupil_logger is not None:
            loggers.append(pupil_logger)

        rosbag_logger = get_rosbag_recorder(log_dir, config)
        if rosbag_logger is not None:
            loggers.append(rosbag_logger)
    
    return loggers


def log_trial_init(goals, config):
    log_dir = config.get(LOGGING_CONFIG_NAME, {}).get('data_dir', None)
    if log_dir is None:
        return

    # make sure the directory exists
    if not os.path.isdir(log_dir):
        os.makedirs(log_dir)
    with open(os.path.join(log_dir, 'goals.yaml'), 'w') as f:
        yaml.safe_dump( { g.name: g.pose.tolist() for g in goals }, f )

    # log the config we used to run this trial
    # we can't log the env or robot, so make a copy
    new_config = dict(config)
    del new_config['env']
    del new_config['robot']
    with open(os.path.join(log_dir, 'config.yaml'), 'w') as f:
        yaml.safe_dump(new_config, f)

    


