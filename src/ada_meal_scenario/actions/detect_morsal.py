import numpy, os, rospy, time, json
from bypassable_action import ActionException, BypassableAction
from std_msgs.msg import String
from catkin.find_in_workspaces import find_in_workspaces

import logging
logger = logging.getLogger('ada_meal_scenario')

#name which we will add indices to
morsal_base_name = 'morsal'
def morsal_index_to_name(ind):
    return morsal_base_name + str(ind)

delta_base_name = 'delta'
def delta_index_to_name(ind):
    return delta_base_name + str(ind)


class DetectMorsal(BypassableAction):
    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DetectBite', bypass=bypass)


    def _run(self, robot, timeout=None):
        self.remove_morsals_next_indices(robot.GetEnv(), 0)
        
        m_detector = MorsalDetector(robot)
        m_detector.start()

        # Now wait for the morsal to be detected
        env = robot.GetEnv()
        logger.info('Waiting to detect morsal')
        start_time = time.time()
        time.sleep(1.0) # give time for the camera image to stabilize
        while not env.GetKinBody(morsal_index_to_name(0)) and (timeout is None or time.time() - start_time < timeout):
            logger.info('Waiting for detections')
            time.sleep(1.0)

        #filter bad detections
        logger.info('Getting morsals in env')
        all_morsals = GetAllMorsalsInEnv(env)
        all_deltas = GetAllMorsalsInEnv(env, name_func=delta_index_to_name)
        #inds_to_filter = FilterMorsalsOnTable(env.GetKinBody('table'), all_morsals)
        #self.filter_morsal_inds(env, inds_to_filter, all_morsals)
        logger.info('projecting morsal on table')
        ProjectMorsalsOnTable(env.GetKinBody('table'), all_morsals)
        ProjectDeltasToMorsals(env.GetKinBody('table'), all_deltas, all_morsals)

        logger.info('stopping detector')
        m_detector.stop()

        if not env.GetKinBody(morsal_index_to_name(0)):
            raise ActionException(self, 'Failed to detect any morsals.')

    def _bypass(self, robot, num_morsals=3):
        m_detector = MorsalDetector(robot)
        for i in range(num_morsals):
            # Here we want to place the kinbody
            #  somewhere in the environment
            morsal_in_camera = numpy.eye(4)
            #morsal_in_camera[:3,3] = [0.1, 0., 0.25]
            morsal_in_camera[:3,3] = [0.02, -0.02, 0.52]

            #add random noise
            rand_max_norm = 0.15
            morsal_in_camera[0:2, 3] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm

            #switch to this if you want to test noise in world frame, not camera frame
            camera_in_world = robot.GetLink('Camera_Depth_Frame').GetTransform()
            morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)
#            morsal_in_world[0:2, 3] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm
            #morsal_in_world[2,3] -= 0.17
            morsal_in_camera = numpy.dot(numpy.linalg.inv(camera_in_world), morsal_in_world)

            m_detector.add_morsal(morsal_in_camera, 
                                  morsal_index_to_name(i),
                                  delta_index_to_name(i))

        env = robot.GetEnv()
        all_morsals = GetAllMorsalsInEnv(env)
        all_deltas = GetAllMorsalsInEnv(env, name_func=delta_index_to_name)
        num_morsals_before_filter = len(all_morsals)
        
        ProjectMorsalsOnTable(env.GetKinBody('table'), all_morsals)

        # MAP THE DELTAS TO THE PRESTAB POSITION
        ProjectDeltasToMorsals(env.GetKinBody('table'), all_deltas, all_morsals)
        inds_to_filter = FilterMorsalsOnTable(env.GetKinBody('table'), all_morsals)
        self.filter_morsal_inds(env, inds_to_filter, all_morsals)

        #remove the kinbodies we used in previous timesteps not used here
        self.remove_morsals_next_indices(robot.GetEnv(), num_morsals, end_ind=num_morsals_before_filter)

    def filter_morsal_inds(self, env, inds_to_filter, all_morsals):
        """ Removes the OpenRAVE kin bodies for all morsals with index in inds_to_filter
        Also renames to ensure morsals in the environment have consecutive order

        @param env the OpenRAVE environment
        @param inds_to_filter indices to remove
        @param all_morsals list of all morsels currently in environment
        """
        #remove filtered morsals from env
        morsals_to_remove = [v for i,v in enumerate(all_morsals) if i in inds_to_filter]
        for morsal_to_remove in morsals_to_remove:
            env.Remove(morsal_to_remove)
        all_morsals = [v for i,v in enumerate(all_morsals) if i not in inds_to_filter]
        
        #rename to make sure consecutive order
        for ind,morsal in enumerate(all_morsals):
            morsal.SetName(morsal_index_to_name(ind))
        
    def remove_morsals_next_indices(self, env, start_ind, end_ind=0):
        """ Removes the OpenRAVE kin bodies for all morsals with index at
        or greater than the start index
        If end index is specified, will remove morsals up to that index
        Otherwise, will check indices until no morsals with index i is in the environment

        @param env the OpenRAVE environment
        @param start_ind the index to start checking
        @param end_ind the (optional) index to check morsals up until
        """

        ind = start_ind
        morsal_body = env.GetKinBody(morsal_index_to_name(ind))
        while morsal_body or ind < end_ind:
            if morsal_body:
                env.Remove(morsal_body)
            ind+=1
            morsal_body = env.GetKinBody(morsal_index_to_name(ind))

class MorsalDetector(object):    
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.sub = None

        #keep track of hypotheses for morsal locations
        self.morsal_pos_hypotheses = []
        self.morsal_pos_hypotheses_counts = []
        #require this many consecutive detections to add morsals
        self.min_counts_required_addmorsals = 5
        #once that threshhold is reached for any morsal, require this many counts per morsal to add
        self.min_counts_required = 3
        #if less then this treshhold distance, count as consecutive
        self.distance_thresh_count = 0.02

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

    def add_morsal(self, morsal_in_camera, morsal_name=None, delta_name=None):
        camera_in_world = self.robot.GetLink('Camera_Depth_Frame').GetTransform()
        morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)
        import openravepy
        h1 = openravepy.misc.DrawAxes(self.env, camera_in_world)
        h2 = openravepy.misc.DrawAxes(self.env, morsal_in_world)
        
        if morsal_name is None:
            morsal_name = 'morsal'
            delta_name = 'delta'
        
        object_base_path = find_in_workspaces(
            search_dirs=['share'],
            project='ada_meal_scenario',
            path='data',
            first_match_only=True)[0]
        ball_path = os.path.join(object_base_path, 'objects', 'smallsphere.kinbody.xml')
        delta_path = os.path.join(object_base_path, 'objects', 'mediumsphere.kinbody.xml')
        if self.env.GetKinBody(morsal_name) is None:
            with self.env:
                morsal = self.env.ReadKinBodyURI(ball_path)
                delta = self.env.ReadKinBodyURI(delta_path)
                morsal.SetName(morsal_name)
                delta.SetName(delta_name)
                self.env.Add(morsal)
                self.env.Add(delta)
                morsal.Enable(False)
                delta.Enable(False)
        else:
            morsal = self.env.GetKinBody(morsal_name)
            delta = self.env.GetKinBody(delta_name)
        morsal.SetTransform(morsal_in_world)
        delta.SetTransform(morsal_in_world)

    def _callback(self, msg):
        logger.info('Received detection')
        obj =  json.loads(msg.data)
        pts_arr = obj['pts3d']
        morsal_positions = numpy.asarray(pts_arr)
        if(morsal_positions is None) or(len(morsal_positions)==0):
            return

        next_hypoths = []
        next_hypoth_counts = []
        for morsal_pos in morsal_positions:
          dists_all_hypotheses = [numpy.linalg.norm(h - morsal_pos) for h in self.morsal_pos_hypotheses]

          #if none of the distances less then thresh, count as a new detection
          if len(dists_all_hypotheses) == 0 or min(dists_all_hypotheses) > self.distance_thresh_count:
            next_hypoths.append(morsal_pos)
            next_hypoth_counts.append(1)
          else:
            ind = numpy.argmin(dists_all_hypotheses)
            #average with old pos
            old_count = self.morsal_pos_hypotheses_counts[ind]
            new_hypoth_pos = (self.morsal_pos_hypotheses[ind]*old_count + morsal_pos) / (float(old_count + 1))

            next_hypoths.append(new_hypoth_pos)
            next_hypoth_counts.append(old_count+1)
          
        self.morsal_pos_hypotheses = next_hypoths
        self.morsal_pos_hypotheses_counts = next_hypoth_counts

        #if any detection exceeds min count, add all detections with that count
        if max(self.morsal_pos_hypotheses_counts) >= self.min_counts_required_addmorsals:
          logger.info('adding all the morsals')
          morsal_index = 0
          for hypoth_pos, hypoth_pos_count in zip(self.morsal_pos_hypotheses, self.morsal_pos_hypotheses_counts):
            logger.info('checking morsal at pos ' + str(hypoth_pos))
            if hypoth_pos_count >= self.min_counts_required:
              logger.info('adding morsal at pos ' + str(hypoth_pos))
              morsal_in_camera = numpy.eye(4)
              morsal_in_camera[:3,3] = hypoth_pos
              self.add_morsal(morsal_in_camera, 
                              morsal_index_to_name(morsal_index),
                              delta_index_to_name(morsal_index))
              morsal_index += 1
        
def ProjectMorsalsOnTable(table, morsals, dist_above_table=0.03):
    """ Sets all morsals to be the specified distance above the table

    @param table the table kinbody
    @param morsals list of all morsals to project
    @param dist_above_table distance you want the bottom of the morsal to be above the table
    """
    all_morsal_dists = GetAllDistsTableToObjects(table, morsals)
    for dist,morsal in zip(all_morsal_dists, morsals):
        morsal_transform = morsal.GetTransform()
        morsal_transform[2,3] -= dist - dist_above_table
        morsal.SetTransform(morsal_transform)
    
def ProjectDeltasToMorsals(table, deltas, morsals):
    for morsal, delta in zip(morsals, deltas):
        delta_transform = delta.GetTransform()
        morsal_transform = morsal.GetTransform()
        delta_transform[2,3] = morsal_transform[2,3] + 0.06
        delta.SetTransform(delta_transform)
    
def FilterMorsalsOnTable(table, morsals, thresh_dist_below_table=0.0, thresh_dist_above_table=0.1):
    """ Detects all morsals either below the table by more then the specified amount, or above
    by more then the specified amount, and returns their indices

    @param table the table kinbody
    @param morsals list of all morsals to project
    @param thresh_dist_below_table threshhold distance where we want to filter if the bottom of the morsal
        is below the table by more then this amount
    @param thresh_dist_above_table threshhold distance where we want to filter if the bottom of the morsal
        is above the table by more then this amount

    @return indices of morsals either below the table more then threshhold, or above by more then threshhold
    """
    all_morsal_dists = GetAllDistsTableToObjects(table, morsals)
    inds_to_filter = []
    for ind,dist in enumerate(all_morsal_dists):
        if dist < thresh_dist_below_table or dist > thresh_dist_above_table:
            inds_to_filter.append(ind)

    return inds_to_filter


def GetAllDistsTableToObjects(table, objects):
    """ Get the distance between the top of the table and the bottom of each object

    @param table the table kinbody
    @param objects list of all objects 
    @return the distance between the bottom of each object and the top of the table
    """
    table_aabb = table.ComputeAABB()
    top_of_table = table_aabb.pos()[2] + table_aabb.extents()[2]
    dists = []
    for obj in objects:
        object_aabb = obj.ComputeAABB()
        bottom_of_object = object_aabb.pos()[2] - object_aabb.extents()[2]
        dist_diff = (bottom_of_object - top_of_table)
        dists.append(dist_diff)
    return dists


def GetAllMorsalsInEnv(env, start_ind=0, end_ind=0, name_func=morsal_index_to_name):
    """ Tries to get all the morsals in the environment, based on naming
    Assumes the function morsal_index_to_name was used to name all morsals
    And all morsal numbers are consecutive

    @param env the OpenRAVE environment
    @param start_ind the index to start checking
    @param end_ind the (optional) index to check morsals up until. If unspecifed, 
            will continue to check until no bodies are found
    """
    
    all_morsals = []
    ind = start_ind
    morsal_body = env.GetKinBody(name_func(ind))
    while morsal_body or ind < end_ind:
        if morsal_body:
            all_morsals.append(morsal_body)
        ind+=1
        morsal_body = env.GetKinBody(name_func(ind))
    return all_morsals
