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
from ada_meal_scenario.assistance.assistance_config import get_ada_handler_config, is_autonomous
from ada_meal_scenario.loggers.loggers import get_loggers, log_trial_init, get_log_dir
from ada_meal_scenario.trajectory_actions import create_move_robot_to_end_effector_pose_action

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
            if ik_sols is None or len(ik_sols) == 0:
                return False, None
            else:
                return True, ik_sols[0]
    return True, ik_sol

def set_ghost_to_ik(name, robot, ik):
    pass
    # ghost_robot = robot.GetEnv().GetRobot(name)
    # if ghost_robot is None:
    #     ghost_robot = openravepy.RaveCreateRobot(robot.GetEnv(), robot.GetXMLId())
    #     ghost_robot.Clone(robot, openravepy.openravepy_int.CloningOptions.Bodies)
    #     ghost_robot.SetName(name)
    #     ghost_robot.Enable(False)  # don't count collisions
    #     ghost_robot.arm = ghost_robot.GetManipulators()[0]
    #     ghost_robot.SetActiveManipulator(ghost_robot.arm)
    #     robot.GetEnv().Add(ghost_robot)
    #     for link in ghost_robot.GetLinks():
    #         if link in ghost_robot.arm.GetChildLinks():
    #             # part of end-effector
    #             for geom in link.GetGeometries():
    #                 geom.SetTransparency(0.4)
    #         else:
    #             link.SetVisible(False)
    
    # ghost_robot.SetActiveDOFValues(ik)


def check_ik_for_goal(robot, goal):
    logger.info('checking IK for {}'.format(goal.name))
    for tf in goal.target_poses:
        ok, ik = check_ik_for_pose(robot, tf)
        if ok:
            set_ghost_to_ik('ghost_{}'.format(goal.name), robot, ik)
            return True, goal
    return False, goal


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
        
    return ActionSequenceFactory(
            ).then(build_goals
            ).then(make_async_mapper(check_ik_for_goal)
            ).then(filter_goals
            )

@futurize(blocking=True)
def filter_goals(prev_result, config, *args, **kwargs):
    ok_goals = []
    for ok, goal in prev_result:
        if ok:
            ok_goals.append(goal)
        else:
            logger.warning('removed {}: no valid IK found'.format(goal.name))
            config['env'].Remove(config['env'].GetKinBody(goal.name))
    return ok_goals

def move_to_goal(prev_result, config, status_cb, goal_idx=None):
    goals = prev_result
    # collect async loggers
    # AdaHandler handles logging its own data in-thread
    # but loggers that just need to start and stop are passed as separate objects
    loggers = get_loggers(goals, config)

    if goal_idx is None:
        goal_idx = np.random.randint(len(goals))
    true_goal = goals[goal_idx]
    status_cb('Autonomously going to goal {} ({})'.format(goal_idx, true_goal.name))

    # TODO: make the normal one log like this too
    @futurize(blocking=True)
    def start_logging(*args, **kwargs):
        print('starting log')
        for logger in loggers:
            logger.start()

    @futurize(blocking=True)
    def stop_logging(*args, **kwargs):
        print('stopping log')
        for logger in loggers:
            logger.stop()

    @futurize(blocking=True)
    def return_goals(*args, **kwargs):
        return goals

    return ActionSequenceFactory(
        ).then(start_logging
        ).then(create_move_robot_to_end_effector_pose_action(true_goal.target_poses[0])
        ).then(stop_logging
        ).then(return_goals
        ).run(prev_result, config, status_cb)

def run_direct_teleop(prev_result, config, status_cb):
    status_cb('Starting direct teleop')
    goals = prev_result
    loggers = get_loggers(goals, config)
    cfg = AdaHandlerConfig.create(goals=goals, log_dir=get_log_dir(config), **get_ada_handler_config(config))
    cfg = cfg._replace(direct_teleop_only=True)
    return AdaHandler(config['env'], config['robot'], cfg, loggers)

def run_assistance_on_goals(prev_result, config, status_cb):
    if is_autonomous(config):
        return move_to_goal(prev_result, config, status_cb)
    else:
        status_cb('Starting trial')
        goals = prev_result
        return AdaHandler(
            config['env'], config['robot'],
            AdaHandlerConfig.create(goals=goals, log_dir=get_log_dir(config), **get_ada_handler_config(config)),
            get_loggers(goals, config))

def make_goal_filter_with_assistance(get_robot_pos_fn=get_prestab_position):
    return make_goal_builder(get_robot_pos_fn
        ).then(run_assistance_on_goals)
