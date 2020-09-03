import logging, numpy, prpy, os
import prpy.rave, prpy.util
from bypassable_action import ActionException, BypassableAction
from prpy.planning.base import PlanningError

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

class RunTrajectory(BypassableAction):

    def __init__(self, name, traj_path, traj_filename_base, bypass=False):
        """
        @param name The name of the action
        @param trajfile The full path to the trajectory file
        @param bypass If true, move world to a configuration
          that represents the end of the action
        """
        BypassableAction.__init__(self, name, bypass=bypass)
        self.traj = None
        self.traj_path = traj_path
        self.traj_filename_base = traj_filename_base

    def _load_traj(self, env, robot):
        traj_filename = get_traj_filename(self.traj_path, robot.GetName(), self.traj_filename_base)
        self.traj = prpy.rave.load_trajectory(env, traj_filename)
        
    def _run(self, manip):
        
        robot = manip.GetRobot()
        env = robot.GetEnv()

        if self.traj is None:
            self._load_traj(env, robot)

        cspec = self.traj.GetConfigurationSpecification()
        first_wpt = self.traj.GetWaypoint(0)
        first_config = cspec.ExtractJointValues(first_wpt, robot, manip.GetArmIndices())
        current_config = manip.GetDOFValues()

        fork = env.GetKinBody('fork')
        
        # Plan to the start of the trajectory if we aren't already there
        if not prpy.util.IsAtTrajectoryStart(robot, self.traj):
            logger.info('Planning to start of trajectory for action %s' % self.name)
            print 'Planning to start of trajectory for action ' + self.name
            try:
                robot.PlanToConfiguration(first_config, execute=True)
            except PlanningError, e:
                raise ActionException(self, 'Failed to plan to start of trajectory: %s' % str(e))

        try:
            robot.ExecuteTrajectory(self.traj)
        except e:
            logger.error(str(e))
            logger.error('Executing saved traj failed. Trying to replan to config')
            last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)
            last_config = cspec.ExtractJointValues(last_wpt, robot, manip.GetArmIndices())
            robot.PlanToConfiguration(last_config, execute=True)
            
    def _bypass(self, manip):
        
        robot = manip.GetRobot()
        env = robot.GetEnv()
        if self.traj is None:
            self._load_traj(env)

        cspec = self.traj.GetConfigurationSpecification()
        last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)

        dofindices = prpy.util.GetTrajectoryIndices(self.traj)
        dofvalues = cspec.ExtractJointValues(last_wpt, robot, manip.GetArmIndices())
        
        with env:
            robot.SetDOFValues(values=dofvalues, dofindices=dofindices)
        

class LookAtPlate(RunTrajectory):

    def __init__(self, bypass=False):
        #looking at plate traj
        #starts at: ada_meal_scenario_servingConfiguration
        #end at: ada_meal_scenario_lookingAtPlateConfiguration
        
        from catkin.find_in_workspaces import find_in_workspaces
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if len(traj_path) == 0:
            raise ActionException(self, 'Failed to find trajectory data path')
        
        RunTrajectory.__init__(self, "LOOKING_AT_PLATE", traj_path[0], 'traj_lookingAtPlate.xml', bypass=bypass)

class Serve(RunTrajectory):

    def __init__(self, bypass=False):
        
        from catkin.find_in_workspaces import find_in_workspaces
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if len(traj_path) == 0:
            raise ActionException(self, 'Failed to find trajectory data path')
        
        RunTrajectory.__init__(self, "SERVING", traj_path[0], 'traj_serving.xml', bypass=bypass)


def get_traj_filename(traj_path, robot_name, traj_filename_base):
    return os.path.join(traj_path, 'trajectories', robot_name + '_' + traj_filename_base)


