import numpy, os, rospy, time, json
from bypassable_action import ActionException, BypassableAction
from std_msgs.msg import String
from catkin.find_in_workspaces import find_in_workspaces

import logging
logger = logging.getLogger('ada_meal_scenario')

#name which we will add indices to
morsel_base_name = 'morsel'
def morsel_index_to_name(ind):
    return morsel_base_name + str(ind)


delta_base_name = 'delta'
def delta_index_to_name(ind):
    return delta_base_name + str(ind)

class DetectMorsel(BypassableAction):

    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DetectBite', bypass=bypass)


    def _run(self, robot, timeout=None):

        self.remove_morsels_next_indices(robot.GetEnv(), 0)
        
        m_detector = MorselDetector(robot)
        m_detector.start()

        # Now wait for the morsel to be detected
        env = robot.GetEnv()
        logger.info('Waiting to detect morsel')
        start_time = time.time()
        time.sleep(1.0) # give time for the camera image to stabilize
        while not env.GetKinBody(morsel_index_to_name(0)) and (timeout is None or time.time() - start_time < timeout):
            logger.info('Waiting for detections')
            time.sleep(1.0)

        #filter bad detections
        logger.info('Getting morsels in env')
        all_morsels = GetAllMorselsInEnv(env)
        #inds_to_filter = FilterMorselsOnTable(env.GetKinBody('table'), all_morsels)
        #self.filter_morsel_inds(env, inds_to_filter, all_morsels)
        logger.info('projecting morsel on table')
        ProjectMorselsOnTable(env.GetKinBody('table'), all_morsels)

        logger.info('stopping detector')
        m_detector.stop()

        if not env.GetKinBody(morsel_index_to_name(0)):
            raise ActionException(self, 'Failed to detect any morsels.')

    def _bypass(self, robot, num_morsels=3):

        m_detector = MorselDetector(robot)
        for i in range(num_morsels):
            # Here we want to place the kinbody
            #  somewhere in the environment
            morsel_in_camera = numpy.eye(4)
            morsel_in_camera[:3,3] = [0.02, -0.02, 0.52]

            #add random noise
            rand_max_norm = 0.15
            morsel_in_camera[0:2, 3] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm

            #switch to this if you want to test noise in world frame, not camera frame
            camera_in_world = robot.GetLink('Camera_Depth_Frame').GetTransform()
            morsel_in_world = numpy.dot(camera_in_world, morsel_in_camera)
#            morsel_in_world[0:2, 3] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm
            #morsel_in_world[2,3] -= 0.17
            morsel_in_camera = numpy.dot(numpy.linalg.inv(camera_in_world), morsel_in_world)

            m_detector.add_morsel(morsel_in_camera, morsel_index_to_name(i))

        env = robot.GetEnv()
        all_morsels = GetAllMorselsInEnv(env)
        num_morsels_before_filter = len(all_morsels)
        
        ProjectMorselsOnTable(env.GetKinBody('table'), all_morsels)
        inds_to_filter = FilterMorselsOnTable(env.GetKinBody('table'), all_morsels)
        self.filter_morsel_inds(env, inds_to_filter, all_morsels)

        #remove the kinbodies we used in previous timesteps not used here
        self.remove_morsels_next_indices(robot.GetEnv(), num_morsels, end_ind=num_morsels_before_filter)

    def filter_morsel_inds(self, env, inds_to_filter, all_morsels):
        """ Removes the OpenRAVE kin bodies for all morsels with index in inds_to_filter
        Also renames to ensure morsels in the environment have consecutive order

        @param env the OpenRAVE environment
        @param inds_to_filter indices to remove
        @param all_morsels list of all morsels currently in environment
        """
        #remove filtered morsels from env
        morsels_to_remove = [v for i,v in enumerate(all_morsels) if i in inds_to_filter]
        for morsel_to_remove in morsels_to_remove:
            env.Remove(morsel_to_remove)
        all_morsels = [v for i,v in enumerate(all_morsels) if i not in inds_to_filter]
        
        #rename to make sure consecutive order
        for ind,morsel in enumerate(all_morsels):
            morsel.SetName(morsel_index_to_name(ind))
        
    
    def remove_morsels_next_indices(self, env, start_ind, end_ind=0):
        """ Removes the OpenRAVE kin bodies for all morsels with index at
        or greater than the start index
        If end index is specified, will remove morsels up to that index
        Otherwise, will check indices until no morsels with index i is in the environment

        @param env the OpenRAVE environment
        @param start_ind the index to start checking
        @param end_ind the (optional) index to check morsels up until
        """

        ind = start_ind
        morsel_body = env.GetKinBody(morsel_index_to_name(ind))
        while morsel_body or ind < end_ind:
            if morsel_body:
                env.Remove(morsel_body)
            ind+=1
            morsel_body = env.GetKinBody(morsel_index_to_name(ind))


class MorselDetector(object):
    
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.sub = None

        #keep track of hypotheses for morsel locations
        self.morsel_pos_hypotheses = []
        self.morsel_pos_hypotheses_counts = []
        #require this many consecutive detections to add morsels
        self.min_counts_required_addmorsels = 5
        #once that threshhold is reached for any morsel, require this many counts per morsel to add
        self.min_counts_required = 3
        #if less then this treshhold distance, count as consecutive
        self.distance_thresh_count = 0.02

    def start(self):
        logger.info('Subscribing to morsel detection')
        self.sub = rospy.Subscriber("/perception/morsel_detection", 
                                    String, 
                                    self._callback, 
                                    queue_size=1)
    
    def stop(self):
        logger.info('Unsubscribing from morsel detection')
        self.sub.unregister() # unsubscribe
        self.sub = None

    def add_morsel(self, morsel_in_camera, morsel_name=None):
        camera_in_world = self.robot.GetLink('Camera_Depth_Frame').GetTransform()
        morsel_in_world = numpy.dot(camera_in_world, morsel_in_camera)
        import openravepy
        h1 = openravepy.misc.DrawAxes(self.env, camera_in_world)
        h2 = openravepy.misc.DrawAxes(self.env, morsel_in_world)
        
        if morsel_name is None:
            morsel_name = 'morsel'
        
        object_base_path = find_in_workspaces(
            search_dirs=['share'],
            project='ada_meal_scenario',
            path='data',
            first_match_only=True)[0]
        ball_path = os.path.join(object_base_path, 'objects', 'smallsphere.kinbody.xml')
        if self.env.GetKinBody(morsel_name) is None:
            with self.env:
                morsel = self.env.ReadKinBodyURI(ball_path)
                morsel.SetName(morsel_name)
                self.env.Add(morsel)
                morsel.Enable(False)
        else:
            morsel = self.env.GetKinBody(morsel_name)
        morsel.SetTransform(morsel_in_world)



        
    def _callback(self, msg):
        logger.info('Received detection')
        obj =  json.loads(msg.data)
        pts_arr = obj['pts3d']
        morsel_positions = numpy.asarray(pts_arr)
        if(morsel_positions is None) or(len(morsel_positions)==0):
            return

        next_hypoths = []
        next_hypoth_counts = []
        for morsel_pos in morsel_positions:
          dists_all_hypotheses = [numpy.linalg.norm(h - morsel_pos) for h in self.morsel_pos_hypotheses]

          #if none of the distances less then thresh, count as a new detection
          if len(dists_all_hypotheses) == 0 or min(dists_all_hypotheses) > self.distance_thresh_count:
            next_hypoths.append(morsel_pos)
            next_hypoth_counts.append(1)
          else:
            ind = numpy.argmin(dists_all_hypotheses)
            #average with old pos
            old_count = self.morsel_pos_hypotheses_counts[ind]
            new_hypoth_pos = (self.morsel_pos_hypotheses[ind]*old_count + morsel_pos) / (float(old_count + 1))

            next_hypoths.append(new_hypoth_pos)
            next_hypoth_counts.append(old_count+1)
          

        self.morsel_pos_hypotheses = next_hypoths
        self.morsel_pos_hypotheses_counts = next_hypoth_counts

        #if any detection exceeds min count, add all detections with that count
        if max(self.morsel_pos_hypotheses_counts) >= self.min_counts_required_addmorsels:
          logger.info('adding all the morsels')
          morsel_index = 0
          for hypoth_pos, hypoth_pos_count in zip(self.morsel_pos_hypotheses, self.morsel_pos_hypotheses_counts):
            logger.info('checking morsel at pos ' + str(hypoth_pos))
            if hypoth_pos_count >= self.min_counts_required:
              logger.info('adding morsel at pos ' + str(hypoth_pos))
              morsel_in_camera = numpy.eye(4)
              morsel_in_camera[:3,3] = hypoth_pos
              self.add_morsel(morsel_in_camera, morsel_index_to_name(morsel_index))
              morsel_index += 1
        

def ProjectMorselsOnTable(table, morsels, dist_above_table=0.03):
    """ Sets all morsels to be the specified distance above the table

    @param table the table kinbody
    @param morsels list of all morsels to project
    @param dist_above_table distance you want the bottom of the morsel to be above the table
    """
    all_morsel_dists = GetAllDistsTableToObjects(table, morsels)
    for dist,morsel in zip(all_morsel_dists, morsels):
        morsel_transform = morsel.GetTransform()
        morsel_transform[2,3] -= dist - dist_above_table
        morsel.SetTransform(morsel_transform)
    

def FilterMorselsOnTable(table, morsels, thresh_dist_below_table=0.0, thresh_dist_above_table=0.1):
    """ Detects all morsels either below the table by more then the specified amount, or above
    by more then the specified amount, and returns their indices

    @param table the table kinbody
    @param morsels list of all morsels to project
    @param thresh_dist_below_table threshhold distance where we want to filter if the bottom of the morsel
        is below the table by more then this amount
    @param thresh_dist_above_table threshhold distance where we want to filter if the bottom of the morsel
        is above the table by more then this amount

    @return indices of morsels either below the table more then threshhold, or above by more then threshhold
    """
    all_morsel_dists = GetAllDistsTableToObjects(table, morsels)
    inds_to_filter = []
    for ind,dist in enumerate(all_morsel_dists):
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


def GetAllMorselsInEnv(env, start_ind=0, end_ind=0):
    """ Tries to get all the morsels in the environment, based on naming
    Assumes the function morsel_index_to_name was used to name all morsels
    And all morsel numbers are consecutive

    @param env the OpenRAVE environment
    @param start_ind the index to start checking
    @param end_ind the (optional) index to check morsels up until. If unspecifed, 
            will continue to check until no bodies are found
    """
    
    all_morsels = []
    ind = start_ind
    morsel_body = env.GetKinBody(morsel_index_to_name(ind))
    while morsel_body or ind < end_ind:
        if morsel_body:
            all_morsels.append(morsel_body)
        ind+=1
        morsel_body = env.GetKinBody(morsel_index_to_name(ind))
    return all_morsels
