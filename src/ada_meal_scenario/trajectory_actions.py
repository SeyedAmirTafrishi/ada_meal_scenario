
from catkin.find_in_workspaces import find_in_workspaces
import logging, numpy, prpy, os
import prpy.rave, prpy.util
from action_sequence import ActionSequence, defer, futurize, NoOp
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


class RunSavedTrajectory(ActionSequence):
    _TRAJ_PATH = None
    __CACHED_TRAJ__ = {}
    def __init__(self, file_base, traj_status, prev_result=None, config=None, status_cb=None):
        # load up the trajectories
        traj_filename = _get_traj_file(file_base, config)
        if traj_filename not in RunSavedTrajectory.__CACHED_TRAJ__:
            RunSavedTrajectory.__CACHED_TRAJ__[
                traj_filename] = prpy.rave.load_trajectory(config['env'], traj_filename)
        self.traj = RunSavedTrajectory.__CACHED_TRAJ__[traj_filename]
        self.env = config['env']
        self.robot = config['robot']

        if not config.get('skip_motion', False):
            status_cb(traj_status)
            action_factories = [self._move_to_traj_start,
                                self._execute_stored_traj]
        else:
            action_factories = [self._bypass]

        super(RunSavedTrajectory, self).__init__(action_factories=action_factories)

    def _move_to_traj_start(self, prev_result, config, status_cb):
        if prpy.util.IsAtTrajectoryStart(self.robot, self.traj):
            return NoOp()
        else:
            cspec = self.traj.GetConfigurationSpecification()
            first_wpt = self.traj.GetWaypoint(0)
            first_config = cspec.ExtractJointValues(
                first_wpt, self.robot, self.robot.GetActiveManipulator().GetArmIndices())
            traj_to_start = self.robot.PostProcessPath(self.robot.PlanToConfiguration(first_config, defer=True))
            return execute_trajectory(self.robot, traj_to_start)

    def _execute_stored_traj(self, prev_result, config, status_cb):
        try:
            return execute_trajectory(self.robot, self.traj)
        except PlanningError as e:
            logger.error(
                'Executing saved traj failed. Trying to replan to config')
            last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)
            last_config = cspec.ExtractJointValues(
                last_wpt, robot, self.robot.GetActiveManipulator().GetArmIndices())
            traj = self.robot.PostProcessPath(self.robot.PlanToConfiguration(last_config))
            return execute_trajectory(self.robot, traj)
            
    @futurize(blocking=True)
    def _bypass(self, prev_result, config, status_cb):
        cspec = self.traj.GetConfigurationSpecification()
        last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)

        dofindices = prpy.util.GetTrajectoryIndices(self.traj)
        dofvalues = cspec.ExtractJointValues(
            last_wpt, self.robot, self.robot.GetActiveManipulator().GetArmIndices())
        
        with self.env:
            self.robot.SetDOFValues(values=dofvalues, dofindices=dofindices)
        

# load in the expected configurations
def _get_traj_file(file_base, config):
    if RunSavedTrajectory._TRAJ_PATH is None:
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if not traj_path:
            raise RuntimeError('Failed to find saved trajectory files')
        RunSavedTrajectory._TRAJ_PATH = traj_path[0]
    
    return os.path.join(RunSavedTrajectory._TRAJ_PATH, 'trajectories', '{}_{}'.format(config['robot'].GetName(), file_base))
    

LookAtPlate = partial(RunSavedTrajectory, 'traj_lookingAtPlate.xml', 'Moving to look at plate')
Serve = partial(RunSavedTrajectory, 'traj_serving.xml', 'Serving')




