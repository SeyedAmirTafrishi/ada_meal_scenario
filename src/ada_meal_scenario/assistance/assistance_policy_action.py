# creates a shared autonomy policy that runs essentially as an action

from collections import OrderedDict
import logging
import numpy as np
import openravepy
import os
import rospkg

from ada_assistance_policy.AdaHandler import AdaHandler, AdaHandlerConfig
from ada_assistance_policy.Goal import Goal
from ada_teleoperation.DataRecordingUtils import TrajectoryData
from ada_meal_scenario.gui_frames import OptionSelector

try:
    import Tkinter as tk
except ImportError:  # python 3
    import tkinter as tk


project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

class AssistanceConfigFrame(tk.Frame, object):
    CONFIG_NAME = 'ada_handler'

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
        return {AssistanceConfigFrame.CONFIG_NAME: config}

    def set_state(self, state):
        self.method_selector.set_state(state)
        self.device_selector.set_state(state)
        self.transition_function_selector.set_state(state)
        self.prediction_selector.set_state(state)


def get_prestab_position(env, robot, morsel, offsets=np.zeros((2, 1))):
    fork = env.GetKinBody('fork')
    if fork is None:
        desired_ee_pose = np.array(
            [[-0.06875708, 0.25515971, -0.96445113, 0.51087426],
             [0.2036257, 0.9499768, 0.23681355, 0.03655854],
             [0.97663147, -0.18010443, -0.11727471, 0.92], [0., 0., 0., 1.]])
    else:
        desired_fork_tip_in_world = np.array([[-1., 0., 0., 0.],
                                                 [0., 1., 0., 0.],
                                                 [0., 0., -1., 0.],
                                                 [0., 0., 0., 1.]])

        morsel_pose = morsel.GetTransform()
        zoffset = 0.06

        desired_fork_tip_in_world[:2, 3] = morsel_pose[:2, 3] + offsets
        desired_fork_tip_in_world[2, 3] = morsel_pose[2, 3] + zoffset

        fork_tip_in_world = fork.GetLink('tinetip').GetTransform()
        ee_in_world = robot.GetActiveManipulator().GetEndEffectorTransform()
        ee_in_fork_tip = np.dot(np.linalg.inv(fork_tip_in_world),
                                   ee_in_world)
        desired_ee_pose = np.dot(desired_fork_tip_in_world, ee_in_fork_tip)

    with robot:
        ik_filter_options = openravepy.IkFilterOptions.CheckEnvCollisions
        # first call FindIKSolution which is faster if it succeeds
        ik_sol = robot.GetActiveManipulator().FindIKSolution(desired_ee_pose, ik_filter_options)
        # if it fails, call FindIKSolutions, which is slower but samples other start configurations
        if ik_sol is None:
            ik_sols = robot.GetActiveManipulator().FindIKSolutions(desired_ee_pose, ik_filter_options)
            if ik_sols is None:
                logger.info('Found no iks for morsel ' + \
                            morsel.GetName() + '. Removing from detected list.')
                return None

    logger.info('Found ik for morsel ' + morsel.GetName())
    return desired_ee_pose


def read_offsets_from_file(filename='morsel_offsets.txt'):
    xoffset, yoffset = 0., 0.
    full_filename = rospkg.RosPack().get_path(
        'ada_meal_scenario') + '/' + filename
    if os.path.isfile(full_filename):
        with open(full_filename, 'r') as f:
            for line in f:
                split_line = line.split()
                # first check if second item in split is a number
                try:
                    offset = float(split_line[1])
                    if split_line[0] == 'xoffset':
                        xoffset = offset
                    elif split_line[0] == 'yoffset':
                        yoffset = offset
                    else:
                        logger.info('unrecognized offset from line: ' + line)

                except ValueError:
                    logger.info(
                        'could not read the following line because second value is not a float: '
                        + line)
                    continue
    return xoffset, yoffset


def do_assistance(prev_result, config):
    env = config['env']
    robot = config['robot']

    # we expect the previous action to be some form of DetectGoals
    # which gives a list of KinBodys as their result
    morsel_bodies = prev_result

    xoffset, yoffset = read_offsets_from_file()
    # get_prestab_position is specific to morsels!
    # todo: make this a config or at least easier to modify
    desired_tfs = [
        get_prestab_position(env, robot, morsel, np.array([xoffset, yoffset]))
        for morsel in morsel_bodies
    ]
    goals = [
        Goal(morsel.GetTransform(), [tf])
        for morsel, tf in zip(morsel_bodies, desired_tfs) if tf is not None
    ]

    # Log the morsels to file
    # if filename_trajdata is not None:
    #     filename_morsels = os.path.splitext(filename_trajdata)[
    #                             0] + '_morsels.yaml'
    #     with open(filename_morsels, 'wb') as f:
    #         yaml.dump({m.GetName(): m.GetTransform().tolist()
    #                    for m in morsel_bodies}, f)

    return AdaHandler(
        config['env'], config['robot'],
        AdaHandlerConfig.create(goals=goals, **config[AssistanceConfigFrame.CONFIG_NAME]))
