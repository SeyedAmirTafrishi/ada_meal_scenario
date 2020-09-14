
from catkin.find_in_workspaces import find_in_workspaces
import logging, numpy, prpy, os
import prpy.rave, prpy.util
from action_sequence import ActionSequence, defer, futurize, NoOp
from prpy.planning.base import PlanningError
from functools import partial

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)


class RunTrajectory(ActionSequence):
    _TRAJ_PATH = None
    __CACHED_TRAJ__ = {}
    def __init__(self, file_base, traj_status, prev_result=None, config=None, status_cb=None):
        # load up the trajectories
        traj_filename = _get_traj_file(file_base, config)
        if traj_filename not in RunTrajectory.__CACHED_TRAJ__:
            RunTrajectory.__CACHED_TRAJ__[
                traj_filename] = prpy.rave.load_trajectory(config['env'], traj_filename)
        self.traj = RunTrajectory.__CACHED_TRAJ__[traj_filename]
        self.env = config['env']
        self.robot = config['robot']

        if not config.get('skip_motion', False):
            status_cb(traj_status)
            action_factories = [self._move_to_traj_start,
                                self._execute_stored_traj]
        else:
            action_factories = [self._bypass]

        super(RunTrajectory, self).__init__(action_factories=action_factories)

    def _move_to_traj_start(self, prev_result, config, status_cb):
        if prpy.util.IsAtTrajectoryStart(self.robot, self.traj):
            return NoOp()
        else:
            cspec = self.traj.GetConfigurationSpecification()
            first_wpt = self.traj.GetWaypoint(0)
            first_config = cspec.ExtractJointValues(
                first_wpt, self.robot, self.robot.GetActiveManipulator().GetArmIndices())
            traj_to_start = self.robot.PostProcessPath(self.robot.PlanToConfiguration(first_config, defer=True))
            return self._execute_traj(traj_to_start)

    def _execute_traj(self, traj):
        if self.robot.simulated:
            # if the robot is simulated, it calls prpy.base.robot.ExecuteTrajectory()
            # which does NOT support defer=True
            return defer(blocking=False, target=self.robot.ExecuteTrajectory, args=(traj,))
        else:
            return self.robot.ExecuteTrajectory(traj, defer=True)

    def _execute_stored_traj(self, prev_result, config, status_cb):
        try:
            return self._execute_traj(self.traj)
        except PlanningError as e:
            logger.error(
                'Executing saved traj failed. Trying to replan to config')
            last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)
            last_config = cspec.ExtractJointValues(
                last_wpt, robot, self.robot.GetActiveManipulator().GetArmIndices())
            traj = self.robot.PostProcessPath(self.robot.PlanToConfiguration(last_config))
            return self._execute_traj(traj)
            
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
    if RunTrajectory._TRAJ_PATH is None:
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if not traj_path:
            raise RuntimeError('Failed to find saved trajectory files')
        RunTrajectory._TRAJ_PATH = traj_path[0]
    
    return os.path.join(RunTrajectory._TRAJ_PATH, 'trajectories', '{}_{}'.format(config['robot'].GetName(), file_base))
    

LookAtPlate = partial(RunTrajectory, 'traj_lookingAtPlate.xml', 'Moving to look at plate')
Serve = partial(RunTrajectory, 'traj_serving.xml', 'Serving')




