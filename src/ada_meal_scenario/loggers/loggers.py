
import copy
import logging
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
from ada_meal_scenario.loggers.goal_transform_publisher import get_goal_transform_publisher

LOGGING_CONFIG_NAME = 'logging'

logger = logging.getLogger('ada_meal_scenario')


def check_dir(base_dir, uid):
    return not os.path.isdir(get_filename(base_dir, uid))
def get_filename(base_dir, uid):
  return os.path.join(base_dir, 'user_') + str(uid).zfill(3)

def get_next_available_user_ind(base_dir, lo=-1, hi=float('inf')):
    # this is waaaaaay too fancy and bug-prone
    # but i wanted to implement binary search
    # and you can't stop me
    assert hi > lo
    if lo < 0:
        if check_dir(base_dir, 0):
            return 0
        else:
            return get_next_available_user_ind(base_dir, lo=0, hi=hi)
    elif hi == float('inf'):
        q = lo+1
        while not check_dir(base_dir, q):
            q = q*2
        return get_next_available_user_ind(base_dir, lo=lo, hi=q)
    elif hi == lo+1:
        return hi
    else:
        q = int( (lo + hi) / 2)
        if check_dir(base_dir, q):
            return get_next_available_user_ind(base_dir, lo=lo, hi=q)
        else:
            return get_next_available_user_ind(base_dir, lo=q, hi=hi)


class LoggingOptions(tk.Frame, object):
    __USER_ID_INVALID_BG__ = "#cc0000"
    def __init__(self, parent, initial_config, run_fn):
        super(LoggingOptions, self).__init__(parent)
        initial_config = initial_config.get(LOGGING_CONFIG_NAME, {})
        self._initial_config = initial_config

        self.left_column = tk.Frame(self)
        self.left_column.grid(row=0, column=0, sticky='nesw')
        self.left_column.columnconfigure(0, weight=1)
        self.right_column = tk.Frame(self)
        self.right_column.grid(row=0, column=1, sticky='nesw')
        self.right_column.columnconfigure(0, weight=1)

        self.rowconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)


        self.logging_frame = tk.LabelFrame(self.left_column, text='Location')
        base_dir = initial_config.get('base_dir', 
                os.path.join(rospkg.RosPack().get_path('ada_meal_scenario'), 'trajectory_data'))
        
        # choose top dir for logging
        self.data_root_var = tk.StringVar(value=base_dir)
        self.data_root_label = tk.Label(
            self.logging_frame, textvariable=self.data_root_var, wraplength=300, anchor='w')
        self.data_root_label.grid(row=0, column=0, columnspan=2, sticky='nwe')
        self.data_root_label.bind('<Configure>', lambda e: e.widget.configure(wraplength=e.widget.winfo_width()))
        self.data_root_button = tk.Button(self.logging_frame, text='Select data directory', command=self._set_data_root)
        self.data_root_button.grid(row=1, column=0, columnspan=2, sticky='nw')

        # choose user id
        self.user_id_var = tk.StringVar()
        self.user_id_var.set(initial_config.get('data_dir', ''))
        self.user_id_var.trace("w", self._validate_user_id)
        self.user_id_entry = tk.Entry(
            self.logging_frame, textvariable=self.user_id_var)
        self.user_id_entry.grid(row=2, column=1, sticky=tk.N+tk.E+tk.W)
        self.user_id_label = tk.Label(self.logging_frame, text='User ID')
        self.user_id_label.grid(row=2, column=0, sticky=tk.N+tk.W)
        self.user_id_auto_var = tk.IntVar()
        self.user_id_auto_var.set(initial_config.get('auto_user_id', True))
        self.user_id_auto_check = tk.Checkbutton(self.logging_frame, variable=self.user_id_auto_var, text='Auto-increment ID', command=self._set_user_id_auto)
        self.user_id_auto_check.grid(row=3, column=0, columnspan=2, sticky='nw')

        # initialize fields
        self.user_id_orig_bg = self.user_id_entry.cget("bg")
        self._set_user_id_auto()
        self.logging_frame.columnconfigure(1, weight=1)

        self.logging_frame.grid(row=0, column=0, padx=1, pady=1, sticky='nesw')
        self.loggers = []
        self.added_logger_names = []

    def add_logger_frame(self, frame_fn, logger_fn=None, side='left', name=None, **kwargs):
        if frame_fn is not None:
            if side == 'left':
                parent = self.left_column
            elif side == 'right':
                parent = self.right_column
            else:
                parent = self.logging_frame
            frame = frame_fn(parent, self._initial_config)
            self.loggers.append(frame)

            grid_args = dict(column=0, padx=1, pady=1, sticky='nesw')
            grid_args.update(kwargs)
            frame.grid(**grid_args)
        else:
            frame = None

        if name is None:
            if frame_fn:
                name = frame_fn.__name__
            elif logger_fn:
                name = logger_fn.__name__
            else:
                raise ValueError("one of frame_fn, logger_fn must not be None")
        if logger_fn:
            _register_logger(name, logger_fn)
            self.added_logger_names.append(name)

        return frame

    def _set_data_root(self):
        data_root = tkfile.askdirectory(initialdir=self.data_root_var.get(), title='Choose root directory for logging')
        if data_root is not None:
            self.data_root_var.set(data_root)
            self._update_user_id()

    def _get_data_dir(self):
        return get_filename(self.data_root_var.get(), self.user_id_var.get())
    
    def _validate_user_id(self, *_):
        data_dir = self._get_data_dir()
        if os.path.exists(data_dir):
            self.user_id_entry.config(bg=LoggingOptions.__USER_ID_INVALID_BG__)
        else:
            self.user_id_entry.config(bg=self.user_id_orig_bg)

    def _set_user_id_auto(self, *args):
        if self.user_id_auto_var.get():
            self.user_id_entry.configure(state=tk.DISABLED)
            self._update_user_id()
        else:
            self.user_id_entry.configure(state=tk.NORMAL)

    def _update_user_id(self):
        # when we've finished a trial, we need to advance the user id
        next_user_id = get_next_available_user_ind(self.data_root_var.get())
        self.user_id_var.set(next_user_id)

    def get_config(self):
        data_dir = self._get_data_dir()
        if os.path.exists(data_dir):
            raise ValueError("Directory exists: {}".format(data_dir))
        base_res = {
            'base_dir': self.data_root_var.get(),
            'data_dir': data_dir,
            'auto': bool(self.user_id_auto_var.get()),
            'added_loggers': self.added_logger_names
        }
        for logger in self.loggers:
            base_res.update(logger.get_config())

        return { LOGGING_CONFIG_NAME: base_res }

    def set_state(self, state):
        self.data_root_button.configure(state=state)
        if not self.user_id_auto_var.get():
            self.user_id_entry.configure(state=state)
        self.user_id_auto_check.configure(state=state)

        for logger in self.loggers:
            logger.set_state(state=state)
        
        # this is a hacky signal that says a trial probably just ended
        # so re-check the log id
        # this is a no-op if the directory was never used
        if self.user_id_auto_var.get():
            self._update_user_id()

def build_logger_frame(*args, **kwargs):
    from ada_meal_scenario.loggers.goal_transform_publisher import GoalTransformPublisherConfigFrame
    from ada_meal_scenario.loggers.pupil_recorder import PupilRecorderConfigFrame, get_pupil_recorder
    from ada_meal_scenario.loggers.rosbag_recorder import RosbagRecorderConfigFrame, get_rosbag_recorder
    from ada_meal_scenario.loggers.zed_remote_recorder import RemoteRecorderConfigFrame, get_zed_remote_recorder
    from ada_meal_scenario.loggers.zed_node_recorder import ZedNodeRecorderConfigFrame, get_zed_node_recorder
    from ada_meal_scenario.loggers.notes_recorder import NotesLoggerFrame

    logger_frame = LoggingOptions(*args, **kwargs)
    logger_frame.add_logger_frame(PupilRecorderConfigFrame, get_pupil_recorder, side='left')
    logger_frame.add_logger_frame(RemoteRecorderConfigFrame, get_zed_remote_recorder, side='left')
    logger_frame.add_logger_frame(GoalTransformPublisherConfigFrame, side='left')  # TODO: fix signature
    logger_frame.add_logger_frame(RosbagRecorderConfigFrame, get_rosbag_recorder, side='right')
    logger_frame.add_logger_frame(ZedNodeRecorderConfigFrame, get_zed_node_recorder, side='left')
    logger_frame.add_logger_frame(NotesLoggerFrame, side='left')

    return logger_frame


def get_log_dir(config):
    return config.get(LOGGING_CONFIG_NAME, {}).get('data_dir', None)

_ALL_LOGGERS = {}
# boo globals but what can you do, i'm not sure where else to put this data
def _register_logger(name, new_logger):
    if name not in _ALL_LOGGERS:
        _ALL_LOGGERS[name] = new_logger
    else:
        logger.warn('Trying to re-register logger: {}'.format(name))

def get_loggers(goals, config):
    if LOGGING_CONFIG_NAME not in config:
        return []

    log_trial_init(goals, config)  # TODO: make this a "logger"?

    config = config[LOGGING_CONFIG_NAME]
    log_dir = config.get('data_dir', None)
    if log_dir is not None:
        # make sure the directory exists
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)

    loggers = []

    for logger_name in config['added_loggers']:
        logger.info("Initializing logger {}".format(logger_name))
        log_obj = _ALL_LOGGERS[logger_name](log_dir, config)
        if log_obj is not None:
            loggers.append(log_obj)

    # TODO: match the signature so we can register this like the others
    goal_transform_publisher = get_goal_transform_publisher(goals)
    if goal_transform_publisher is not None:
        loggers.append(goal_transform_publisher)
    
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

    
def disable_logging(cfg):
    config = cfg.copy()
    if LOGGING_CONFIG_NAME in config:
        config[LOGGING_CONFIG_NAME]['data_dir'] = None
    return config

