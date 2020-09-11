from collections import OrderedDict

try:
    import Tkinter as tk
except ImportError:  # python 3
    import tkinter as tk

from ada_meal_scenario.gui_frames import OptionSelector


ASSISTANCE_CONFIG_NAME = 'ada_handler'

class AssistanceConfigFrame(tk.Frame, object):

    _METHOD_SELECTOR_VALS = OrderedDict([
        [
            'Direct Teleop', {
                'direct_teleop_only': True,
                'blend_only': False,
                'fix_magnitude_user_command': False,
                'gamma': 0.,
                'pick_goal': False
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

    _DEVICE_SELECTOR_VALS = OrderedDict([
        ['Mouse', {
            'input_interface_name': 'mouse',
            'num_input_dofs': 2
        }],
        [
            'Kinova USB', {
                'input_interface_name': 'kinova',
                'num_input_dofs': 2
            }
        ],
    ])

    _TRANSITION_FUNCTION_SELECTOR_VALUES = OrderedDict(
        [['a+u', lambda g: lambda a, u: a + u],
         ['gamma*a + (1-gamma)*u', lambda g: lambda a, u: g * a + (1 - g) * u],
         [
             '2*gamma*a + (2-2*gamma)*u',
             lambda g: lambda a, u: 2 * g * a + 2 * (1 - g) * u
         ]])

    _PREDICTOR_SELECTOR_VALUES = OrderedDict([
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

    def __init__(self, parent):
        super(AssistanceConfigFrame, self).__init__(parent)

        sticky = tk.N+tk.S+tk.E+tk.W

        self.method_selector = OptionSelector(
            self,
            title="Method:",
            option_names=AssistanceConfigFrame._METHOD_SELECTOR_VALS.keys(),
            option_values=AssistanceConfigFrame._METHOD_SELECTOR_VALS.values(),
            default_selection=4)
        self.method_selector.grid(row=0,
                                  column=0,
                                  padx=2,
                                  pady=2,
                                  sticky=sticky)

        self.device_selector = OptionSelector(
            self,
            title="UI Device:",
            option_names=AssistanceConfigFrame._DEVICE_SELECTOR_VALS.keys(),
            option_values=AssistanceConfigFrame._DEVICE_SELECTOR_VALS.values(),
            default_selection=1)
        self.device_selector.grid(row=0,
                                  column=1,
                                  padx=2,
                                  pady=2,
                                  sticky=sticky)

        self.transition_function_selector = OptionSelector(
            self,
            title="Transition Function:",
            option_names=AssistanceConfigFrame.
            _TRANSITION_FUNCTION_SELECTOR_VALUES.keys(),
            option_values=AssistanceConfigFrame.
            _TRANSITION_FUNCTION_SELECTOR_VALUES.values())
        self.transition_function_selector.grid(row=0,
                                               column=2,
                                               padx=2,
                                               pady=2,
                                               sticky=sticky)

        self.prediction_selector = OptionSelector(
            self,
            title="Prediction Method",
            option_names=AssistanceConfigFrame._PREDICTOR_SELECTOR_VALUES.keys(
            ),
            option_values=AssistanceConfigFrame._PREDICTOR_SELECTOR_VALUES.
            values(),
            default_selection=0)
        self.prediction_selector.grid(row=0,
                                      column=3,
                                      padx=2,
                                      pady=2,
                                      sticky=sticky)

    def get_config(self):
        config = {}
        config.update(self.method_selector.get_value())
        config.update(self.device_selector.get_value())
        config[
            'transition_function'] = self.transition_function_selector.get_value(
            )(config['gamma'])
        del config['gamma'] # we only use gamma for the transition function, don't pass it directly
        config.update(self.prediction_selector.get_value())
        return {ASSISTANCE_CONFIG_NAME: config}

    def set_state(self, state):
        self.method_selector.set_state(state)
        self.device_selector.set_state(state)
        self.transition_function_selector.set_state(state)
        self.prediction_selector.set_state(state)
