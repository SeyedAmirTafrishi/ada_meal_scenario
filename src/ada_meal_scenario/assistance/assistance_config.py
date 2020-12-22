from collections import OrderedDict
from ada_teleoperation.UserInputMapper import list_profiles

try:
    import Tkinter as tk
except ImportError:  # python 3
    import tkinter as tk

from ada_meal_scenario.gui_frames import OptionSelector


ASSISTANCE_CONFIG_NAME = 'ada_handler'


_METHOD_SELECTOR_VALS = OrderedDict([
    [
        'Direct Teleop', {
            'direct_teleop_only': True,
            'blend_only': False,
            'fix_magnitude_user_command': False,
            'gamma': 0.,
            'pick_goal': False,
        }
    ],
    [
        'Shared Auton Lvl 1', {
            'direct_teleop_only': False,
            'blend_only': False,
            'fix_magnitude_user_command': False,
            'gamma': 0.33,
            'pick_goal': False
        }
    ],
    [
        'Shared Auton Lvl 2', {
            'direct_teleop_only': False,
            'blend_only': False,
            'fix_magnitude_user_command': False,
            'gamma': 0.67,
            'pick_goal': False
        }
    ],
    [
        'Full Auton User Goal', {
            'direct_teleop_only': False,
            'blend_only': False,
            'fix_magnitude_user_command': False,
            'gamma': 1.,
            'pick_goal': False
        }
    ],
    [
        'Full Auton Random Goal', {
            'direct_teleop_only': False,
            'blend_only': False,
            'fix_magnitude_user_command': False,
            'gamma': 1.,
            'pick_goal': True
        }
    ],
])

_TRANSITION_FUNCTION_SELECTOR_VALS = OrderedDict(
    [['a+u', lambda g: lambda a,u: a+u,
    ],
    ['gamma*a + (1-gamma)*u', lambda g: lambda a,u: g*a+(1-g)*u,
    ],
    ['2*gamma*a + (2-2*gamma)*u', lambda g: lambda a,u: 2*g*a+2*(1-g)*u
    ]])

_PREDICTOR_SELECTOR_VALS = OrderedDict([
    ['Policy', {
        'predict_policy': True,
        'predict_gaze': False
    }],
    ['Gaze', {
        'predict_policy': False,
        'predict_gaze': True
    }],
    ['Merged', {
        'predict_policy': True,
        'predict_gaze': True
    }],
])

class AssistanceConfigFrame(tk.Frame, object):


    def __init__(self, parent, initial_config):
        super(AssistanceConfigFrame, self).__init__(parent)

        initial_config = initial_config.get(ASSISTANCE_CONFIG_NAME, {})

        sticky = tk.N+tk.S+tk.E+tk.W

        self.method_selector = OptionSelector(
            self,
            title="Method:",
            option_names=_METHOD_SELECTOR_VALS.keys(),
            default_selection=initial_config.get('method_index', 4))
        self.method_selector.grid(row=0,
                                  column=0,
                                  padx=2,
                                  pady=2,
                                  sticky=sticky)
        
        self.input_profile_selector = OptionSelector(
            self,
            title='Input Profile:',
            option_names=list_profiles(),
            default_selection=0
        )
        self.input_profile_selector.grid(row=0, column=1, padx=2, pady=2, sticky=sticky)

        self.transition_function_selector = OptionSelector(
            self,
            title="Transition Function:",
            option_names=_TRANSITION_FUNCTION_SELECTOR_VALS.keys(),
            default_selection=initial_config.get('transition_function_index', 0))
        self.transition_function_selector.grid(row=0,
                                               column=2,
                                               padx=2,
                                               pady=2,
                                               sticky=sticky)

        self.prediction_selector = OptionSelector(
            self,
            title="Prediction Method",
            option_names=_PREDICTOR_SELECTOR_VALS.keys(),
            default_selection=initial_config.get('prediction_index', 0))
        self.prediction_selector.grid(row=0,
                                      column=3,
                                      padx=2,
                                      pady=2,
                                      sticky=sticky)

    def get_config(self):
        config = {
            'method': self.method_selector.get_value(),
            'transition_function': self.transition_function_selector.get_value(),
            'prediction': self.prediction_selector.get_value(),
            'input_profile_name': self.input_profile_selector.get_value()
        }
        return {ASSISTANCE_CONFIG_NAME: config}

    def set_state(self, state):
        self.method_selector.set_state(state)
        self.transition_function_selector.set_state(state)
        self.prediction_selector.set_state(state)
        self.input_profile_selector.set_state(state)

def get_ada_handler_config(config):
    config = config[ASSISTANCE_CONFIG_NAME]

    out = {
        'input_profile_name': config['input_profile_name']
    }
    out.update(_METHOD_SELECTOR_VALS[config['method']])
    out.update(_PREDICTOR_SELECTOR_VALS[config['prediction']])

    
    # lambda is bad in the config directly bc we want to save/load it
    # so use a string lookup and resolve it on trial start
    out['transition_function'] = _TRANSITION_FUNCTION_SELECTOR_VALS[config['transition_function']](out['gamma'])
    del out['gamma']

    # clear out the stuff we used to save 
    return out
