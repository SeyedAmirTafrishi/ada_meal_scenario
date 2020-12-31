# creates a shared autonomy policy that runs essentially as an action

import logging
import numpy as np
import openravepy
import os
import rospkg

from ada_assistance_policy.AdaHandler import AdaHandler, AdaHandlerConfig
from ada_assistance_policy.Goal import Goal
from ada_teleoperation.DataRecordingUtils import TrajectoryData
from ada_meal_scenario.action_sequence import make_async_mapper, ActionSequenceFactory, futurize
from ada_meal_scenario.assistance.assistance_config import get_ada_handler_config
from ada_meal_scenario.loggers.loggers import get_loggers, log_trial_init, get_log_dir

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)


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


def get_prestab_position(env, robot, morsel):
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

    return desired_ee_pose


def check_ik_for_pose(robot, desired_ee_pose):
    with robot:
        ik_filter_options = openravepy.IkFilterOptions.CheckEnvCollisions
        # first call FindIKSolution which is faster if it succeeds
        # we want to pass releasegil so the gui doesn't freeze
        # but for whatever reason the py overload doesn't work unless we pass in [] for free_pos
        ik_sol = robot.GetActiveManipulator().FindIKSolution(desired_ee_pose, [], ik_filter_options, False, True)  # release GIL so GUI doesn't freeze
        # if it fails, call FindIKSolutions, which is slower but samples other start configurations
        if ik_sol is None:
            ik_sols = robot.GetActiveManipulator().FindIKSolutions(desired_ee_pose, [], ik_filter_options, False, True)  # release GIL so GUI doesn't freeze
            if ik_sols is None:
                return False
    return True

def check_ik_for_goal(robot, goal):
    logger.info('checking IK for {}'.format(goal.name))
    return (any(check_ik_for_pose(robot, tf) for tf in goal.target_poses), goal)


def make_goal_builder(get_robot_pos_fn=get_prestab_position):
    @futurize(blocking=True)
    def build_goals(prev_result, config, status_cb):
        """
        prev_result: iterable of KinBody-s representing goal objects
        """
        status_cb('Preparing assistance')
        env = config['env']
        robot = config['robot']
        goal_bodies = prev_result

        # apply the manual offset
        # todo: move this to the gui
        xoffset, yoffset = read_offsets_from_file()
        for goal in goal_bodies:
            tf = goal.GetTransform()
            tf[:2,3] += [xoffset, yoffset]
            goal.SetTransform(tf)

        goals = [
            (robot, Goal(goal.GetName(), goal.GetTransform(), [get_robot_pos_fn(env, robot, goal)]))
            for goal in goal_bodies
        ]

        logger.debug('got {} goals'.format(len(goals)))
        return goals
        
    return ActionSequenceFactory([build_goals]
            ).then(make_async_mapper(check_ik_for_goal)
            ).then(filter_goals)

@futurize(blocking=True)
def filter_goals(prev_result, config, *args, **kwargs):
    ok_goals = []
    for ok, goal in prev_result:
        if ok:
            ok_goals.append(goal)
        else:
            logger.warning('removed {}: no valid IK found'.format(goal.GetName()))
            config['env'].Remove(goal)
    return ok_goals

def run_assistance_on_goals(prev_result, config, status_cb):
    true_goals = prev_result
    # collect async loggers
    # AdaHandler handles logging its own data in-thread
    # but loggers that just need to start and stop are passed as separate objects
    loggers = get_loggers(true_goals, config)

    status_cb('Starting trial')
    return AdaHandler(
        config['env'], config['robot'],
        AdaHandlerConfig.create(goals=true_goals, log_dir=get_log_dir(config), **get_ada_handler_config(config)),
        loggers)


def make_goal_filter_with_assistance(get_robot_pos_fn=get_prestab_position):
    return make_goal_builder(get_robot_pos_fn
        ).then(run_assistance_on_goals)