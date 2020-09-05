import logging, numpy, prpy, os
import prpy.rave, prpy.util
from action_sequence import ActionSequence, make_future, NoOp
from prpy.planning.base import PlanningError
from functools import partial

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)


class RunTrajectory(ActionSequence):
    __CACHED_TRAJ__ = {}
    def __init__(self, name, traj_path, traj_filename_base, prev_result=None, config=None):
        # load up the trajectories
        traj_filename = os.path.join(
            traj_path, 'trajectories', config['robot'].GetName() + '_' + traj_filename_base)
        if traj_filename not in RunTrajectory.__CACHED_TRAJ__:
            RunTrajectory.__CACHED_TRAJ__[
                traj_filename] = prpy.rave.load_trajectory(config['env'], traj_filename)
        self.traj = RunTrajectory.__CACHED_TRAJ__[traj_filename]
        self.env = config['env']
        self.robot = config['robot']

        if not config.get('skip_motion', False):
            action_factories = [self._move_to_traj_start,
                                self._execute_stored_traj]
        else:
            action_factories = [self._bypass]

        super(RunTrajectory, self).__init__(action_factories=action_factories)
        

    def _move_to_traj_start(self, prev_result, config):
        cspec = self.traj.GetConfigurationSpecification()
        first_wpt = self.traj.GetWaypoint(0)
        first_config = cspec.ExtractJointValues(
            first_wpt, self.robot, self.robot.GetActiveManipulator().GetArmIndices())
        current_config = self.robot.GetActiveManipulator().GetDOFValues()
        if prpy.util.IsAtTrajectoryStart(self.robot, self.traj):
            return NoOp()
        else:
            traj = self.robot.PostProcessPath(self.robot.PlanToConfiguration(first_config, defer=True))
            return self._execute_traj(traj)

    def _execute_traj(self, traj):
        if self.robot.simulated:
            # if the robot is simulated, it calls prpy.base.robot.ExecuteTrajectory()
            # which does NOT support defer()
            return make_future(self.robot.ExecuteTrajectory)(traj)
        else:
            return self.robot.ExecuteTrajectory(traj, defer=True)

    def _execute_stored_traj(self, prev_result, config):
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
            
            
    def _bypass(self, prev_result, config):
        cspec = self.traj.GetConfigurationSpecification()
        last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)

        dofindices = prpy.util.GetTrajectoryIndices(self.traj)
        dofvalues = cspec.ExtractJointValues(
            last_wpt, self.robot, self.robot.GetActiveManipulator().GetArmIndices())
        
        with self.env:
            self.robot.SetDOFValues(values=dofvalues, dofindices=dofindices)
        return NoOp()
        

# load in the expected configurations
from catkin.find_in_workspaces import find_in_workspaces
traj_path = find_in_workspaces(
    search_dirs=['share'],
    project=project_name,
    path='data',
    first_match_only=True)
if len(traj_path) > 0:
    LookAtPlate = partial(RunTrajectory, 'LOOKING_AT_PLATE',
                        traj_path[0], 'traj_lookingAtPlate.xml')
    Serve = partial(RunTrajectory, 'SERVING', traj_path[0], 'traj_serving.xml')
else:
    logger.warning('Failed to find trajectory data path')



