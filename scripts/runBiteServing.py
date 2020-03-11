#!/usr/bin/env python

import adapy, argparse, logging, numpy, os, sys, openravepy, prpy, rospy, random
from prpy.planning.base import PlanningError
from catkin.find_in_workspaces import find_in_workspaces
from ada_meal_scenario.actions.bite_serving import BiteServing
from ada_meal_scenario.actions.bypassable_action import ActionException
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import IPython
import time 

import ada_teleoperation.KinovaStudyHelpers as KinovaStudyHelpers

from prpy.tsr.rodrigues import *

from ada_meal_scenario.gui_handler import *

from gazetracking.pupil_capture import PupilCapture

import warnings
warnings.simplefilter(action = "ignore", category = FutureWarning)


project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

def setup(sim=False, viewer=None, debug=True):
    global robot, serving_phrases

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

    # Load the environment
    env, robot = adapy.initialize(attach_viewer=viewer, sim=sim, env_path=env_path)

    #TODO get this from a rosparam
    right_handed = True

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

    #if sim is True:
    #   startConfig = numpy.array([  3.33066907e-16,   2.22044605e-16,   1.66608370e+00,
    #    -1.65549603e+00,  -1.94424475e-01,   1.06742772e+00,
    #    -1.65409614e+00,   1.30780704e+00])
    logger.info('Setting Initial Robot Configuration to Serving')
    if sim is True:
        #set configuration to look at plate if sim else plan to look at plate
        indices, values = robot.configurations.get_configuration('ada_meal_scenario_servingConfiguration')
        robot.SetDOFValues(dofindices=indices, values=values)
    else:
        robot.arm.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration')
    #    robot.SetDOFValues(startConfig)

    logger.info('Initial Config Set')
    # Load the fork into the robot's hand
    tool = env.ReadKinBodyURI('objects/kinova_tool.kinbody.xml')
    env.Add(tool)
    
    # Fork in end-effector
    #ee_in_world = robot.GetLink('j2n6a300_link_6').GetTransform()
#    tool_in_ee = numpy.array([[ -1., 0.,  0., 0.],
#                              [ 0.,  1., 0., -0.002],
#                              [ 0.,  0.,  -1., -0.118],
#                              [ 0.,  0.,  0., 1.]])


    ee_in_world = robot.arm.GetEndEffectorTransform()
    if right_handed:
        y_trans_tool = 0.004
    else:
        y_trans_tool = -0.004
    tool_in_ee = numpy.array([[ 1., 0.,  0., 0.],
                            [ 0.,  1., 0., y_trans_tool],
                            [ 0.,  0.,  1., -0.042],
                            [ 0.,  0.,  0., 1.]])
    rotate_tool_in_ee = rodrigues([0., 0., 0.])
    rotate_tool_in_ee = rodrigues([0., 0., np.pi/32.])
    #rotate_tool_in_ee = rodrigues([0., 0., -np.pi/32.])
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

    logger.info('creating fork and tool boxes')

    fork_box = make_collision_box_body(fork, add_to_pos=np.array([0.0, 0.0, 0.05]), add_to_extents=np.array([0.02, 0.02, 0.1]))
    tool_box = make_collision_box_body(tool, add_to_pos=np.array([0.0, 0.0, 0.04]), add_to_extents=np.array([0.055, 0.055, 0.055]))

    logger.info('fork and tool boxes created')

    
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

#    print 'grab link: ' + str(grab_link)
#    print 'links to ignore: ' + str(finger_link_inds)
#
#    print 'grabbed name: ' + str(robot.GetGrabbedInfo()[0]._grabbedname)
#    print 'ee name: ' + str(robot.GetGrabbedInfo()[0]._robotlinkname)
#    print 'links ignored' + str(robot.GetGrabbedInfo()[0]._setRobotLinksToIgnore)


    # Set serving phrases
    serving_phrases = ['That looks like a delicious bite ', 
                        'Here you go, I hope you enjoy it ',
                        'That was a good choice ']

    # add boxes for constraint to not hit user
    KinovaStudyHelpers.AddConstraintBoxes(env, robot)
    return env, robot


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
  




def pose_to_arrow_markers(pose, ns='axes', id_start=0, lifetime_secs=10):
  markers = []
  scale_size = 0.1
  dims = [.01, .02, 0.1]
  dirs = []
  dirs.append(np.array([1.,0.,0.]))
  dirs.append(np.array([0.,1.,0.]))
  dirs.append(np.array([0.,0.,1.]))
  for ind,dir in enumerate(dirs):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.ARROW
    marker.action = Marker.ADD;
    marker.id = id_start + ind
    marker.lifetime.secs=lifetime_secs
    marker.ns = ns

    marker.scale.x = dims[0]
    marker.scale.y = dims[1]
    marker.scale.z = dims[2]

    marker.pose.position.x = 0.
    marker.pose.position.y = 0.
    marker.pose.position.z = 0.
    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.

 

    start_pt = pose[0:3,3]
    end_pt = np.dot(pose[0:3,0:3], dir)*scale_size + start_pt
    marker.points = [np_array_to_point(start_pt), np_array_to_point(end_pt)]

    marker.color.r = dir[0]
    marker.color.g = dir[1]
    marker.color.b = dir[2]
    marker.color.a = 1.0

    markers.append(marker)

  return markers

def np_array_to_point(pt_np):
  pt = Point()
  pt.x = pt_np[0]
  pt.y = pt_np[1]
  pt.z = pt_np[2]
  return pt

def joystick_callback(data):
    global joystick_go_signal
    # If the left button is pressed, update go signal flag
    if data.buttons[0]:
        joystick_go_signal = True

#def tasks_callback(data):
#    global serving_phrases
#    # Announce the bite!
#    if data.data == 'SERVING':
#        # Randomly select one of the serving phrases
#        phrase = random.choice(serving_phrases) + username
#        robot.Say(phrase)

def ResetTrial(robot):
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
#      from controller_manager.controller_manager_interface import SwitchController
#      switch_controller_service = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
#      switch_controller_service([], ['traj_controller', 'velocity_joint_mode_controller'], 0)


if __name__ == "__main__":
    global joystick_go_signal, robot, username

    state_pub = rospy.Publisher('ada_tasks',String, queue_size=10)
        
    rospy.init_node('bite_serving_scenario', anonymous=True)

    parser = argparse.ArgumentParser('Ada meal scenario')
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--real", action="store_true", help="Run on real robot (not simulation)")
    parser.add_argument("--viewer", type=str, default='interactivemarker', help="The viewer to load")
    parser.add_argument("--detection-sim", action="store_true", help="Simulate detection of morsal")
    parser.add_argument("--jaco", action="store_true", default=False, help="Using jaco robot")
    parser.add_argument("--userid", type=int, help="User ID number")
    parser.add_argument("--no-pupil-tracking", action="store_true", help="Disable pupil tracking")
    args = parser.parse_args(rospy.myargv()[1:]) # exclude roslaunch args

    sim = not args.real
    env, robot = setup(sim=sim, viewer=args.viewer, debug=args.debug)

    gui_get_event, gui_trial_starting_event, gui_queue, gui_process = start_gui_process(rospy)

    #slow robot down
    #save old limits
#    old_acceleration_limits = robot.GetDOFAccelerationLimits()
#    old_velocity_limits = robot.GetDOFVelocityLimits()
#
#    #slow down robot
#    #robot.SetDOFVelocityLimits(0.6*robot.GetDOFVelocityLimits())
#    #robot.SetDOFAccelerationLimits(0.8*robot.GetDOFAccelerationLimits())
#
#    robot.SetDOFVelocityLimits(old_velocity_limits)
#    robot.SetDOFAccelerationLimits(old_acceleration_limits)
# 

    #start by going to ada_meal_scenario_servingConfiguration
    if sim:
        indices, values = robot.configurations.get_configuration('ada_meal_scenario_servingConfiguration')
        robot.SetDOFValues(dofindices=indices, values=values)
    else:
        robot.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration', execute=True)

#    camera_link = robot.GetLink('Camera_RGB_Frame')
#    camera_transform = camera_link.GetTransform()
#    with prpy.viz.RenderPoses([camera_transform], env):

    # Subscribe to the 'ada_tasks' topic (for talking during certain tasks)
    #task_listener = rospy.Subscriber('ada_tasks', String, tasks_callback)


    # Start eyetracker remote controller
    do_pupil_tracking= not args.no_pupil_tracking
    if do_pupil_tracking:
        logger.info('setting up pupil tracking')
        pupil_capture = PupilCapture()
        pupil_capture.setup(logger)
    else:
        pupil_capture = None

    logger.info('pupil tracker set, or none needed')
    # Where to store rosbags and other user data - set this manually if userid was provided,
    # otherwise dynamically generate it
    from ada_teleoperation.DataRecordingUtils import *
    file_directory_user = None
    file_directory = rospkg.RosPack().get_path('ada_meal_scenario') + '/trajectory_data'
    if args.userid:
        user_number = "%03d"%args.userid
        base_user_filename = "user_" + user_number
        file_directory_user = os.path.join(file_directory, base_user_filename)
        # Check whether the file_directory_user exists
        if os.path.exists(file_directory_user):
            inp = raw_input("Filename " + file_directory_user + " exists. Press q to quit or enter to continue")
            if inp == 'q':
                raise OSError("Filename already exists. Quitting.")
        else:
            os.makedirs(file_directory_user)
    else:
        user_number, file_directory_user = get_next_available_user_ind(file_directory=file_directory, user_folder_base=user_folder_base_default)

    while not rospy.is_shutdown():
        empty_queue(gui_queue)
        gui_get_event.set()
        while gui_queue.empty():
            time.sleep(0.05)
            continue

        gui_return = gui_queue.get()
        if gui_return['quit']:
            break
        elif gui_return['start']:
            if gui_return['record']:
                # Start eyetracker recording
                if pupil_capture:
                    pupil_capture.start()
            #tell gui we are starting to reset next trial
            gui_trial_starting_event.set()
            # Start bite collection and presentation
            try:
                manip = robot.GetActiveManipulator()
                action = BiteServing()
                action.execute(manip, 
                               env, 
                               method=gui_return['method'], 
                               ui_device=gui_return['ui_device'], 
                               state_pub=state_pub, 
                               detection_sim=args.detection_sim, 
                               record_trial=gui_return['record'], 
                               file_directory=file_directory_user,
                               transition_function=gui_return['transition_function'])
            except ActionException, e:
                logger.info('Failed to complete bite serving: %s' % str(e))

                ResetTrial(robot)
                
            except Exception as e:
                logger.error("Other error detected: {}".format(e))
                break

            finally:
                if pupil_capture:
                    pupil_capture.stop()



    gui_process.terminate()

