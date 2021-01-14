import logging
import rospy

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk # python3

from adapy.futures import CancelledError
from ada_meal_scenario.action_sequence import futurize
from gazetracking.pupil_capture import ConfigurablePupilCapture

PUPIL_CONFIG_NAME = 'pupil'

### TODO(rma): Replace all this with gazetracking.gaze_interface_selector
###    to enable either pupil or tobii selection
###    this is a minimal implementation to get back to HARMONIC functionality

@futurize(blocking=False)
def ping_endpoint(endpt):
    try:
        _ = ConfigurablePupilCapture(endpt)
        return True
    except ValueError: # someone (i.e. me) called it that instead of TimeoutError boooo
        return False


class PupilRecorderConfigFrame(tk.LabelFrame, object):
    def __init__(self, parent, initial_config={}):
        super(PupilRecorderConfigFrame, self).__init__(parent, text='Pupil Labs')

        initial_config = initial_config.get(PUPIL_CONFIG_NAME, {})

        self.enabled_var = tk.BooleanVar(value=initial_config.get('enabled', False))
        self.enabled_checkbox = tk.Checkbutton(self, variable=self.enabled_var, text="Enable Pupil Labs recording")
        self.enabled_checkbox.grid(row=0, column=0, columnspan=2, sticky='nw')

        self.endpoint_var = tk.StringVar(value=initial_config.get('endpoint', 'tcp://127.0.0.1:50020'))
        self.endpoint_entry = tk.Entry(self, textvariable=self.endpoint_var)
        self.endpoint_label = tk.Label(self, text='ZMQ endpoint')
        self.endpoint_entry.grid(row=1, column=1, sticky='new')
        self.endpoint_label.grid(row=1, column=0, sticky='nw')

        self.endpoint_check_button = tk.Button(self, text='Check connection', command=self._check_connection)
        self.endpoint_check_var = tk.StringVar()
        self.endpoint_check_label = tk.Label(self, textvariable=self.endpoint_check_var)
        self.endpoint_check_button.grid(row=2, column=0)
        self.endpoint_check_label.grid(row=2, column=1, sticky=tk.W)

    def _check_connection(self):
        self._checker = ping_endpoint(self.endpoint_var.get())
        self.endpoint_check_button.configure(text='Checking...', state=tk.DISABLED)
        # tk doesn't let us use callbacks
        # so we need to poll for the result through tk
        self.after(100, self._update_connection_status)

    def _update_connection_status(self):
        if self._checker.done():
            try:
                if self._checker.result(0.):
                    msg = 'Connection OK'
                else:
                    msg = 'Connection failed'
            except Exception as e:
                msg = 'Connection failed with exception: {}'.format(e)
            self.endpoint_check_var.set(msg)
            self._checker = None
            self.endpoint_check_button.configure(text='Check connection', state=tk.NORMAL)
        else:
            # keep polling
            self.after(100, self._update_connection_status)

    def get_config(self):
        return { PUPIL_CONFIG_NAME: {
            'endpoint': self.endpoint_var.get(),
            'enabled': self.enabled_var.get()
        } }
    
    def set_state(self, state):
        self.endpoint_entry.configure(state=state)
        # The following might have a sync issue with _update_connection_status
        # that means that the button might re-enable after this has been set
        # Could add some fancy lock things... or since checking the endpt has no issues
        # just allow it while the trial is running
        # since this whole class will be replaced with tobii stuff anyway
        # self.endpoint_check_button.configure(state=state)

def get_pupil_recorder(log_dir, config):
    if PUPIL_CONFIG_NAME in config and config[PUPIL_CONFIG_NAME]['enabled']:
        return ConfigurablePupilCapture(config[PUPIL_CONFIG_NAME]['endpoint'])
    else:
        return None
