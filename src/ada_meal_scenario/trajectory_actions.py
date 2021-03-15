
from catkin.find_in_workspaces import find_in_workspaces
import logging, numpy, prpy, os
import prpy.rave, prpy.util
from action_sequence import ActionSequenceFactory, defer, futurize, NoOp, Wait
from prpy.planning.base import PlanningError
from functools import partial

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)


def execute_trajectory(robot, traj):
    if robot.simulated:
        # if the robot is simulated, it calls prpy.base.robot.ExecuteTrajectory()
        # which does NOT support defer=True
        return defer(blocking=False, target=robot.ExecuteTrajectory, args=(traj,))
    else:
        # for some reason if you pass an empty trajectory, defer is ignored and the traj is just sent back
        # so instead return an empty Future
        fut = robot.ExecuteTrajectory(traj, defer=True)
        if hasattr(fut, 'add_done_callback'):
            return fut
        else:
            return NoOp()

def create_trajectory_action(traj):
    def do_move(prev_result=None, config=None, status_cb=None):
        robot = config['robot']
        return execute_trajectory(robot, traj)
    return do_move

def create_move_robot_action(plan_fn):
    def do_move(prev_result=None, config=None, status_cb=None):
        robot = config['robot']
        traj = robot.PostProcessPath(plan_fn(robot))
        return execute_trajectory(robot, traj)
    return do_move

def create_move_robot_to_named_configuration_action(named_config):
    return create_move_robot_action(lambda r: r.PlanToNamedConfiguration(named_config))

def create_move_robot_to_configuration_action(config):
    return create_move_robot_action(lambda r: r.PlanToConfiguration(config))

def create_move_robot_to_end_effector_pose_action(config):
    return create_move_robot_action(lambda r: r.PlanToEndEffectorPose(config))

def create_move_hand_action(value):
    def do_move(prev_result=None, config=None, status_cb=None):
        robot = config['robot']
        return defer(blocking=False, target=robot.arm.hand.CloseHand, args=(value,))
    return do_move


_TRAJ_CACHE = {}
def make_run_saved_trajectory_action(traj_file):
    # need to load the trajectory in the loop bc we need access to the environment
    # so do it once and cache it
    def load_traj(prev_result, config, status_cb):
        logger.info("Loading trajectory {}".format(traj_file))
        if traj_file not in _TRAJ_CACHE:
            _TRAJ_CACHE[traj_file] = prpy.rave.load_trajectory(config['env'], traj_file)
        return NoOp()

    def move_to_traj_start(prev_result, config, status_cb):
        traj = _TRAJ_CACHE[traj_file]
        robot = config['robot']
        if prpy.util.IsAtTrajectoryStart(robot, traj):
            logger.info("Already at trajectory start")
            return NoOp()
        else:
            logger.info("Planning to trajectory start")
            cspec = traj.GetConfigurationSpecification()
            first_wpt = traj.GetWaypoint(0)
            first_config = cspec.ExtractJointValues(
                first_wpt, robot, robot.GetActiveManipulator().GetArmIndices())
            traj_to_start = robot.PostProcessPath(robot.PlanToConfiguration(first_config, defer=True))
            return execute_trajectory(robot, traj_to_start)

    def execute_stored_traj(prev_result, config, status_cb):
        traj = _TRAJ_CACHE[traj_file]
        robot = config['robot']
        try:
            logger.info("Running trajectory")
            return execute_trajectory(robot, traj)
        except PlanningError as e:
            logger.error(
                'Executing saved traj failed. Trying to replan to config')
            last_wpt = traj.GetWaypoint(traj.GetNumWaypoints()-1)
            last_config = cspec.ExtractJointValues(
                last_wpt, robot, robot.GetActiveManipulator().GetArmIndices())
            traj = robot.PostProcessPath(robot.PlanToConfiguration(last_config))
            return execute_trajectory(robot, traj)

    return ActionSequenceFactory(
        ).then(load_traj
        ).then(Wait.factory(0.3)  # give extra time for motion to stop so we don't fail IsAtTrajStart
                                  # really we want a WaitForNoMotion but call that a TODO
        ).then(move_to_traj_start
        ).then(execute_stored_traj)


# load in the expected configurations
_TRAJ_PATH_CACHE = {}
def get_traj_file(file_base, project_name=project_name):
    if project_name not in _TRAJ_PATH_CACHE:
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if not traj_path:
            raise RuntimeError('Failed to find saved trajectory files')
        _TRAJ_PATH_CACHE[project_name] = traj_path[0]
    return os.path.join(_TRAJ_PATH_CACHE[project_name], 'trajectories', file_base)
    

LookAtPlate = make_run_saved_trajectory_action(get_traj_file('ada_traj_lookingAtPlate.xml'))
Serve = make_run_saved_trajectory_action(get_traj_file('ada_traj_serving.xml'))




