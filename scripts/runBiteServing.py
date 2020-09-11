#!/usr/bin/env python

import adapy, argparse, logging, numpy, os, openravepy, prpy, rospy, rospkg, time
import numpy as np
from catkin.find_in_workspaces import find_in_workspaces
from std_msgs.msg import String
from functools import partial

from prpy.planning.base import PlanningError
from prpy.tsr.rodrigues import rodrigues

from ada_meal_scenario.action_sequence import ActionSequence, futurize
from ada_meal_scenario.gui_handler import GuiHandler
from ada_meal_scenario.trajectory_actions import LookAtPlate, Serve
from ada_meal_scenario.detect_goals import GenerateDummyMorsels, DetectMorsels
from ada_meal_scenario.assistance.assistance_policy_action import do_assistance

try:
    from gazetracking.pupil_capture import PupilCapture
except ImportError:
    PupilCapture = None

import warnings
warnings.simplefilter(action = "ignore", category = FutureWarning)

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)



def setup(sim=False, viewer=None, debug=True):
    # load the robot and environment for meal serving

    # find the openrave environment file
    data_base_path = find_in_workspaces(
        search_dirs=['share'],
        project=project_name,
        path='data',
        first_match_only=True)
    if len(data_base_path) == 0:
        raise Exception('Unable to find environment path. Did you source devel/setup.bash?')
    env_path = os.path.join(data_base_path[0], 'environments', 'table.env.xml')
    
    # Initialize logging
    if debug:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    else:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()
    prpy.logger.initialize_logging()

    # Load the environment and robot
    env, robot = adapy.initialize(attach_viewer=viewer, sim=sim, env_path=env_path)

    # Set the active manipulator on the robot
    robot.arm.SetActive()

    # Now set everything to the right location in the environment
    #if using jaco, assume we have the portable mount, and robot should have a different distance to table
    using_jaco = robot.GetName() == 'JACO'
    if using_jaco:
      robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.754],
                              [0., 0., 0., 1.]])
    else:
      robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.795],
                              [0., 0., 0., 1.]])

    with env:
        robot.SetTransform(robot_pose)

    # Set the robot joint configurations
    ResetTrial(robot)

    load_fork_and_tool(env, robot)

    # add boxes for constraint to not hit user
    AddConstraintBoxes(env, robot)
    return env, robot

def load_fork_and_tool(env, robot):
    # Loads the fork and kinova tool holder into the environment, 
    # sets up transforms, and has robot grab the tool
    # also adds objects on those tools, because collision checks with the 
    # meshes were not working properly
    tool = env.ReadKinBodyURI('objects/kinova_tool.kinbody.xml')
    env.Add(tool)
    
    #set the tool and fork transforms
    ee_in_world = robot.arm.GetEndEffectorTransform()
    y_trans_tool = 0.004  #should flip this if we try left_handed
    tool_in_ee = numpy.array([[ 1., 0.,  0., 0.],
                            [ 0.,  1., 0., y_trans_tool],
                            [ 0.,  0.,  1., -0.042],
                            [ 0.,  0.,  0., 1.]])
    rotate_tool_in_ee = rodrigues([0., 0., np.pi/32.])
    tool_in_ee[0:3, 0:3] = np.dot(rotate_tool_in_ee, tool_in_ee[0:3, 0:3])

    tool_in_world = numpy.dot(ee_in_world, tool_in_ee)
    tool.SetTransform(tool_in_world)
    
    fork = env.ReadKinBodyURI('objects/fork.kinbody.xml')
    env.Add(fork)
    fork_in_hole = numpy.array([[1.,0.,0.,0.],
                                [0.,1.,0.,0.],
                                [0.,0.,1.,-0.03],
                                [0.,0.,0.,1.]])
    hole_in_tool = numpy.array([[0.,0.,1.,0.],
                                [0.,1.,0.,-0.0225],
                                [-1.,0.,0.,0.0408],
                                [0.,0.,0.,1.]])
    fork_in_tool = numpy.dot(hole_in_tool, fork_in_hole)                           
    fork_in_ee = numpy.dot(tool_in_ee, fork_in_tool)
    fork_in_world = numpy.dot(ee_in_world, fork_in_ee)
    fork.SetTransform(fork_in_world)


    # since there were problems with the meshes and collision checking, 
    # these boxes were added around the objects. They help make sure 
    # the robot avoids collisions of the fork while it is grabbed
    fork_box = make_collision_box_body(fork, add_to_pos=np.array([0.0, 0.0, 0.05]), add_to_extents=np.array([0.02, 0.02, 0.1]))
    tool_box = make_collision_box_body(tool, add_to_pos=np.array([0.0, 0.0, 0.04]), add_to_extents=np.array([0.055, 0.055, 0.055]))

    #find all finger links
    finger_link_inds = []
    grab_link = None
    for ind,link in enumerate(robot.GetLinks()):
        if 'inger' in link.GetName():
            finger_link_inds.append(ind)
        if 'end_effector' in link.GetName():
            grab_link = link

    robot.arm.hand.CloseHand(1.2)

    logger.info('Grabbing tool and fork')
    robot.Grab(tool, finger_link_inds)
    robot.Grab(fork, finger_link_inds)
    robot.Grab(tool_box, finger_link_inds)
    robot.Grab(fork_box, finger_link_inds)


def make_collision_box_body(kinbody, add_to_pos=np.array([0.0, 0.0, 0.0]), add_to_extents = np.array([0.0, 0.0, 0.0])):
  """Creates a box at position of kin body with optionally modified values
  @param kinbody body we want to create a box around
  @param env openrave environment
  @param add_to_pos modifications to the position of the axis aligned bounding box
  @param add_to_extents modifications to the extents of the axis aligned bounding box
  @return kinbody of box object
  """
  env = kinbody.GetEnv()

  with env:
    old_transform_body = kinbody.GetTransform()
    kinbody.SetTransform(np.eye(4))

    
    kinbody_aabb = kinbody.ComputeAABB()
    box_aabb_arr = np.append(kinbody_aabb.pos()+add_to_pos, kinbody_aabb.extents()+add_to_extents)
    box_aabb_arr = box_aabb_arr.reshape(1,6)
    #print 'aabb box: ' + str(box_aabb_arr)

    box_body = openravepy.RaveCreateKinBody(env, '')
    box_body.SetName(kinbody.GetName() + '_box')
    box_body.InitFromBoxes(box_aabb_arr, False)

    env.Add(box_body)

    kinbody.SetTransform(old_transform_body)
    box_body.SetTransform(old_transform_body)
    
    box_body.Enable(True)

  return box_body

def AddConstraintBoxes(env, robot, handedness='right', name_base="constraint_boxes_", visible=False):
    # Modifies environment to keep the robot inside a defined space
    # Does so by adding invisible boxes around the robot, which planners
    # avoid collisions with

    #add a box behind the robot
    box_behind = openravepy.RaveCreateKinBody(env,'')
    box_behind.SetName(name_base + 'behind')
    box_behind.InitFromBoxes(np.array([[0.,0.,0., 0.4, 0.1, 1.0]]), False)
    env.Add(box_behind)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[1,3] = 0.57
    if handedness == 'right':
        T[0,3] += 0.25
    else:
        T[0,3] -= 0.25
    box_behind.SetTransform(T)


    #add a box above so we don't swing that way too high
    box_above = openravepy.RaveCreateKinBody(env,'')
    box_above.SetName(name_base + 'above')
    box_above.InitFromBoxes(np.array([[0.,0.,0., 0.5, 0.5, 0.1]]), visible)
    env.Add(box_above)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[0,3] += 0.25
    T[1,3] -= 0.25
    T[2,3] += 0.90
    box_above.SetTransform(T)


    box_left = openravepy.RaveCreateKinBody(env,'')
    box_left.SetName(name_base + 'left')
    box_left.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_left)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] += 0.9
    else:
        T[0,3] += 0.25
    T[1,3] = 0.25
    box_left.SetTransform(T)

    box_right = openravepy.RaveCreateKinBody(env,'')
    box_right.SetName(name_base + 'right')
    box_right.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_right)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] -= 0.25
    else:
        T[0,3] -= 0.9
    T[1,3] = 0.25
    box_right.SetTransform(T)
    



def ResetTrial(robot):
  # set the robot to the start configuration for next trial
  logger.info('Resetting Robot')
  if robot.simulated:
      indices, values = robot.configurations.get_configuration('ada_meal_scenario_servingConfiguration')
      robot.SetDOFValues(dofindices=indices, values=values)
  else:
    #first try to plan to serving
    try:
      robot.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration', execute=True)
    except PlanningError, e:
      logger.info('Failed to plan to start config')
      #if it doesn't work, unload controllers


class DummyTask(adapy.futures.Future):
    def __init__(self, prev_result, config):
        super(DummyTask, self).__init__()
        print('Starting dummy task')
        self.timer = rospy.Timer(rospy.Duration(3.), lambda _: self.set_result(None), oneshot=True)

    def cancel(self):
        self.timer.shutdown()
        self.set_cancelled()

def error_task(prev_result, config):
    if config['method'] == 'direct':
        raise RuntimeError(":(((")
    else:
        return DummyTask(prev_result, config)


DummyTrial = partial(ActionSequence, action_factories=[DummyTask, error_task])
LookThenServeTrial = partial(
    ActionSequence, action_factories=[LookAtPlate, Serve])

@futurize(blocking=True)
def PrintResult(prev_result, config):
    rospy.loginfo(prev_result)

LookDetectServeTrial = partial(
    ActionSequence, action_factories=[LookAtPlate, DetectMorsels, PrintResult, Serve])

LookDetectAssistServeTrial = partial(
    ActionSequence, action_factories=[LookAtPlate, DetectMorsels, do_assistance, Serve])
LookGenerateAssistServeTrial = partial(
    ActionSequence, action_factories=[LookAtPlate, GenerateDummyMorsels, do_assistance, Serve])


    

if __name__ == "__main__":
        
    rospy.init_node('bite_serving_scenario', anonymous=True)

    parser = argparse.ArgumentParser('Ada meal scenario')
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--real", action="store_true", help="Run on real robot (not simulation)")
    parser.add_argument("--viewer", type=str, default='interactivemarker', help="The viewer to load")
    parser.add_argument("--detection-sim", action="store_true", help="Simulate detection of morsal")
    parser.add_argument("--jaco", action="store_true", default=False, help="Using jaco robot")
    args = parser.parse_args(rospy.myargv()[1:]) # exclude roslaunch args

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    sim = not args.real
    env, robot = setup(sim=sim, viewer=args.viewer, debug=args.debug)
    base_config = {'env': env, 'robot': robot}


    #start by going to ada_meal_scenario_servingConfiguration
    if sim:
        indices, values = robot.configurations.get_configuration('ada_meal_scenario_servingConfiguration')
        robot.SetDOFValues(dofindices=indices, values=values)
    else:
        robot.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration', execute=True)

    # Set up the GUI
    if args.detection_sim:
        trial_type = LookGenerateAssistServeTrial
    else:
        trial_type = LookDetectAssistServeTrial
    gui = GuiHandler(base_config=base_config, start_trial_callback=trial_type,
                     quit_callback=lambda: rospy.signal_shutdown("Quit button pressed"))

    # Main loop
    while not rospy.is_shutdown():
        gui.run_once()
        # let other tasks run
        # warning: if you use rospy.sleep here, we are incompatible with use_sim_time!
        # (if rospy had roscpp's wallclock this wouldn't be a problem)
        # so just sleep the old way
        time.sleep(0.01)

        # #signal to gui that we want the currently selected options
        # empty_queue(gui_queue)
        # gui_get_event.set()
        # while gui_queue.empty():
        #     time.sleep(0.05)
        #     continue

        # gui_return = gui_queue.get()

        # if gui_return['quit']:
        #     break
        # elif gui_return['start']:
        #     if gui_return['record']:
        #         # Start eyetracker recording
        #         if pupil_capture:
        #             pupil_capture.start()
        #     #tell gui we are starting to reset next trial
        #     gui_trial_starting_event.set()
        #     # Start bite collection and presentation
        #     try:
        #         manip = robot.GetActiveManipulator()
        #         action = BiteServing()
        #         action.execute(manip, 
        #                        env, 
        #                        method=gui_return['method'], 
        #                        ui_device=gui_return['ui_device'], 
        #                        state_pub=state_pub, 
        #                        detection_sim=args.detection_sim, 
        #                        record_trial=gui_return['record'], 
        #                        file_directory=file_directory_user,
        #                        transition_function=gui_return['transition_function'],
        #                        prediction_option= gui_return['prediction_option'])
        #     except ActionException, e:
        #         logger.info('Failed to complete bite serving: %s' % str(e))

        #         ResetTrial(robot)

        #     finally:
        #         if pupil_capture:
        #             pupil_capture.stop()


