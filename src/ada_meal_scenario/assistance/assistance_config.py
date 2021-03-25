from collections import OrderedDict
from ada_teleoperation.UserInputMapper import list_profiles
import ada_assistance_policy.GoalPredictor
import ada_assistance_policy.GazeBasedPredictor

try:
    import Tkinter as tk
except ImportError:  # python 3
    import tkinter as tk
import ttk

from ada_meal_scenario.gui_frames import OptionSelector

ASSISTANCE_CONFIG_NAME = 'assistance_config'

_DIRECT_TELEOP = "Direct teleop"
_SHARED_AUTO = "Shared autonomy"
_FULL_AUTO = "Full autonomy"
_SHARED_AUTO_FIXED = "Shared autonomy (fixed magnitude)"
_BLEND = "Blend"

def _validate_float(val):
    if not val:
        return True
    try:
        float(val)
    except ValueError:
        return False
    else:
        return True

class AssistanceConfigFrame(tk.Frame, object):
    def __init__(self, parent, initial_config, run_fn):
        super(AssistanceConfigFrame, self).__init__(parent)

        initial_config = initial_config.get(ASSISTANCE_CONFIG_NAME, {})

        self.method_selector = OptionSelector(
            self,
            title="Method:",
            option_names=[
                _DIRECT_TELEOP,
                _SHARED_AUTO,
                _FULL_AUTO,
                _SHARED_AUTO_FIXED,
                _BLEND
            ],
            default_selection=initial_config.get('method', 0),
            command=self._update_method)
        self.method_selector.grid(row=0,
                                  column=0,
                                  padx=2,
                                  pady=2,
                                  sticky="new")
        
        self._tf_frame = tk.Frame(self)
        self._tf_frame.grid(row=1, column=0, sticky="nsew")
        self._tf_label = tk.Label(self._tf_frame, text="u_f = 2*g*r*a + 2*(1-g)*u)")
        self._tf_label.grid(row=0, column=0, columnspan=2, sticky="nw")
        self._gamma_label = tk.Label(self._tf_frame, text='g:')
        self._gamma_val = tk.StringVar()
        self._gamma_val.set(str(initial_config.get("g", 0.5)))
        self._gamma_entry = ttk.Combobox(self._tf_frame, textvariable=self._gamma_val, values=["0.5", "0.33", "0.67"])
        self.register(_validate_float)
        self._gamma_entry.configure(validate='all', validatecommand=(_validate_float, "%P"))
        self._gamma_label.grid(row=1, column=0, sticky="nw")
        self._gamma_entry.grid(row=1, column=1, sticky="new")

        self._r_label = tk.Label(self._tf_frame, text='r:')
        self._r_val = tk.StringVar()
        self._r_val.set(str(initial_config.get("r", 1.)))
        self._r_entry = ttk.Combobox(self._tf_frame, textvariable=self._r_val, values=[])
        self._r_entry.configure(validate='all', validatecommand=(_validate_float, "%P"))
        self._r_label.grid(row=2, column=0, sticky="nw")
        self._r_entry.grid(row=2, column=1, sticky="new")

        self.input_profile_selector = OptionSelector(
            self,
            title='Input Profile:',
            option_names=list_profiles(),
            default_selection=initial_config.get("input_profile", 0)
        )
        self.input_profile_selector.grid(row=0, column=1, padx=2, pady=2, sticky="new")

        self._pred_frame = tk.Frame(self)
        self._pred_frame.grid(row=0, column=2, rowspan=3, sticky="nesw")

        self._pred_policy_config = ada_assistance_policy.GoalPredictor.PolicyPredictorConfigFrame(self._pred_frame, initial_config.get("prediction", {}))
        self._pred_policy_config.pack(side=tk.TOP, anchor="nw", fill=tk.X, expand=True)
        self._pred_gaze_config = ada_assistance_policy.GazeBasedPredictor.GazeBasedPredictorConfigFrame(self._pred_frame, initial_config.get("prediction", {}))
        self._pred_gaze_config.pack(side=tk.TOP, anchor="nw", fill=tk.X, expand=True)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)

    def _update_method(self):
        # enable/disable stuff based on method
        cur_method = self.method_selector.get_value()
        shared_auto = cur_method in [ _SHARED_AUTO, _SHARED_AUTO_FIXED ]
        self._gamma_entry.configure(state=tk.NORMAL if shared_auto else tk.DISABLED)
        self._r_entry.configure(state=tk.NORMAL if shared_auto else tk.DISABLED)

    def get_config(self):
        pred_config = dict()
        pred_config.update(self._pred_policy_config.get_config())
        pred_config.update(self._pred_gaze_config.get_config())
        return { ASSISTANCE_CONFIG_NAME: {
            'method': self.method_selector.get_value(),
            'g': float(self._gamma_val.get()),
            'r': float(self._r_val.get()),
            'prediction': pred_config,
            'input_profile': self.input_profile_selector.get_value(),
        } }

    def set_state(self, state):
        self.method_selector.set_state(state)
        self._gamma_entry.configure(state=state)
        self._r_entry.configure(state=state)
        self._update_method()  # update the _gamma state to match
        self.input_profile_selector.set_state(state)
        self._pred_policy_config.set_state(state)
        self._pred_gaze_config.set_state(state)

def get_ada_handler_config(config):
    config = config.get(ASSISTANCE_CONFIG_NAME, {})
    g = config.get('g', 0.5)
    r = config.get('r', 1.)

    return {
        'input_profile_name': config.get('input_profile', ''),
        'direct_teleop_only': config.get('method', '') == _DIRECT_TELEOP,
        'blend_only': config.get('method', '') == _BLEND,
        'pick_goal': config.get('method', '') == _FULL_AUTO,
        'fix_magnitude_user_command': config.get('method', '') == _SHARED_AUTO_FIXED,
        'transition_function': lambda a,u: 2*g*r*a + 2*(1-g)*u,
        'prediction_config': config.get('prediction', {})
    }

def is_autonomous(config):
    config = config.get(ASSISTANCE_CONFIG_NAME, {})
    return config.get('method', '') == _FULL_AUTO

def as_autonomous(config):
    cfg = config.copy()
    cfg[ASSISTANCE_CONFIG_NAME]['method'] = _FULL_AUTO
    return cfg

def as_direct_teleop(config):
    cfg = config.copy()
    cfg[ASSISTANCE_CONFIG_NAME]['method'] = _DIRECT_TELEOP
    return cfg
