# creates a shared autonomy policy that runs essentially as an action

import logging
import numpy as np
import openravepy
import os
import rospkg

from ada_assistance_policy.AdaHandler import AdaHandler, AdaHandlerConfig
from ada_assistance_policy.Goal import Goal
from ada_teleoperation.DataRecordingUtils import TrajectoryData
from ada_meal_scenario.action_sequence import make_async_mapper
from ada_meal_scenario.assistance.assistance_config import ASSISTANCE_CONFIG_NAME
from ada_meal_scenario.loggers.zed_remote_recorder import get_zed_remote_recorder
from ada_meal_scenario.loggers.pupil_recorder import get_pupil_recorder
from ada_meal_scenario.loggers.rosbag_recorder import get_rosbag_recorder

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

    return desired_ee_pose


def check_ik_for_pose(env, robot, desired_ee_pose):
    with robot:
        ik_filter_options = openravepy.IkFilterOptions.CheckEnvCollisions
        # first call FindIKSolution which is faster if it succeeds
        ik_sol = robot.GetActiveManipulator().FindIKSolution(desired_ee_pose, ik_filter_options)
        # if it fails, call FindIKSolutions, which is slower but samples other start configurations
        if ik_sol is None:
            ik_sols = robot.GetActiveManipulator().FindIKSolutions(desired_ee_pose, ik_filter_options)
            if ik_sols is None:
                return False
    return True



def do_assistance(prev_result, config):
    logging.debug('starting assistance function')
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
        for morsel, tf in zip(morsel_bodies, desired_tfs)
    ]

    logging.debug('got {} morsels'.format(len(goals)))
    def run_assistance_on_goals(prev_result, config):
        print('got result: {}'.format(prev_result))
        # prev_result is returned by make_async_mapper below
        # which is a list of results of check_ik_for_pose results
        assert len(prev_result) == len(goals)
        # filter the morsels by the filter
        true_goals = []
        for goal, ok in zip(goals, prev_result):
            if ok:
                true_goals.append(goal)
            else:
                logger.warning('Failed to find IK for morsel, removing')
        

        # Log the morsels to file
        # if filename_trajdata is not None:
        #     filename_morsels = os.path.splitext(filename_trajdata)[
        #                             0] + '_morsels.yaml'
        #     with open(filename_morsels, 'wb') as f:
        #         yaml.dump({m.GetName(): m.GetTransform().tolist()
        #                    for m in morsel_bodies}, f)

        # collect async loggers
        # AdaHandler handles logging its own data in-thread
        # but loggers that just need to start and stop are passed as separate objects
        log_dir = config['logging']['data_dir']
        if log_dir is not None:
            # make sure the directory exists
            if not os.path.isdir(log_dir):
                os.makedirs(log_dir)

            loggers = []

            zed_logger = get_zed_remote_recorder(log_dir, config['logging'])
            if zed_logger is not None:
                loggers.append(zed_logger)

            pupil_logger = get_pupil_recorder(log_dir, config['logging'])
            if pupil_logger is not None:
                loggers.append(pupil_logger)

            rosbag_logger = get_rosbag_recorder(log_dir, config['logging'])
            if rosbag_logger is not None:
                loggers.append(rosbag_logger)

            class DebugLogger:
                def __init__(self): pass
                def start(self): print('Starting debug logger')
                def stop(self): print('Stopping debug logger')
            loggers.append(DebugLogger())

        return AdaHandler(
            config['env'], config['robot'],
            AdaHandlerConfig.create(goals=goals, **config[ASSISTANCE_CONFIG_NAME]),
            loggers)


    # Actually make sure we can do IK for all the goals
    # This can take a little while so make it an ActionSequence
    return make_async_mapper(check_ik_for_pose, 
                    ( (env, robot, tf) for tf in desired_tfs) 
                ).then(run_assistance_on_goals
                ).run(config=config)
