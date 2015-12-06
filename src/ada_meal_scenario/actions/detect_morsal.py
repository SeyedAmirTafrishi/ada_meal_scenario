import numpy, os, rospy, time, json
from bypassable_action import ActionException, BypassableAction
from std_msgs.msg import String
from catkin.find_in_workspaces import find_in_workspaces

import logging
logger = logging.getLogger('ada_meal_scenario')

class DetectMorsal(BypassableAction):

    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DetectBite', bypass=bypass)

    def _run(self, robot, timeout=None):
        
        m_detector = MorsalDetector(robot)
        m_detector.start()

        # Now wait for the morsal to be detected
        env = robot.GetEnv()
        logger.info('Waiting to detect morsal')
        start_time = time.time()
        while not env.GetKinBody('morsal') and (timeout is None or time.time() - start_time < timeout):
            time.sleep(1.0)

        m_detector.stop()

        if not env.GetKinBody('morsal'):
            raise ActionException(self, 'Failed to detect morsal.')

    def _bypass(self, robot):

        # Here we want to place the kinbody
        #  somewhere in the environment
        morsal_in_camera = numpy.eye(4)
        #morsal_in_camera[:3,3] = [0.1, 0., 0.25]
        #morsal_in_camera[:3,3] = [0.05, -0.04, 0.45]
        morsal_in_camera[:3,3] = [0.0, -0.0, 0.45]       
        m_detector = MorsalDetector(robot)
        m_detector.add_morsal(morsal_in_camera)

class MorsalDetector(object):
    
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.sub = None

    def start(self):
        logger.info('Subscribing to morsal detection')
        self.sub = rospy.Subscriber("/perception/morsel_detection", 
                                    String, 
                                    self._callback, 
                                    queue_size=1)
    
    def stop(self):
        logger.info('Unsubscribing from morsal detection')
        self.sub.unregister() # unsubscribe
        self.sub = None

    def add_morsal(self, morsal_in_camera):
        camera_in_world = self.robot.GetLink('Camera_RGB_Frame').GetTransform()
        morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)
        import openravepy
        h1 = openravepy.misc.DrawAxes(self.env, camera_in_world)
        h2 = openravepy.misc.DrawAxes(self.env, morsal_in_world)

        
        object_base_path = find_in_workspaces(
            search_dirs=['share'],
            project='ada_meal_scenario',
            path='data',
            first_match_only=True)[0]
        ball_path = os.path.join(object_base_path, 'objects', 'smallsphere.kinbody.xml')
        if self.env.GetKinBody('morsal') is None:
           morsal = self.env.ReadKinBodyURI(ball_path)
           morsal.SetName('morsal')
           self.env.Add(morsal)
        else:
           morsal = self.env.GetKinBody('morsal')

        morsal_in_world_sim = numpy.array([[-0.45422936, -0.88641709,  0.08910912,  0.5839203 ],
       [-0.88296718,  0.43462921, -0.17738774, -0.01419981],
       [ 0.1185101 , -0.15925514, -0.98009854,  0.80090827],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
	
	#30 degrees
	temp = numpy.array([[ 0.        ,  0.        , -1.        ,  0.46670763],
                                              [ 1.        ,  0.        ,  0.        ,  0.02193009],
                                              [ 0.        , -1.        ,  0.        ,  0.98467413],
                                              [ 0.        ,  0.        ,  0.        ,  1.        ]])
        angle_30 = 120.0/180*numpy.pi
	scaling	 = 0.1;
	trans = numpy.dot(temp, [[scaling * numpy.cos(angle_30)] , [0] , [scaling * numpy.sin(angle_30)] , [1]])
	
	morsal_in_world_sim[0,3] = trans[0];	
	morsal_in_world_sim[1,3] = trans[1];	
	morsal_in_world_sim[2,3] = 0.80090827;	
	morsal_in_world_sim[3,3] = trans[3];	

        morsal.SetTransform(morsal_in_world_sim)
        #morsal.SetTransform(morsal_in_world)


        
    def _callback(self, msg):
        logger.debug('Received detection')
        obj =  json.loads(msg.data)
        pts_arr = obj['pts3d']
        morsal_pos = numpy.asarray(pts_arr)
        if(morsal_pos is None) or(len(morsal_pos)==0):
            return

        morsal_in_camera = numpy.eye(4)
        morsal_in_camera[:3,3] = morsal_pos[0]

        #check 
        self.add_morsal(morsal_in_camera)
        
