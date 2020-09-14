import numpy, os, rospy, time, json
from std_msgs.msg import String
from catkin.find_in_workspaces import find_in_workspaces
from adapy.futures import Future
from functools import partial
from action_sequence import futurize

import logging
logger = logging.getLogger('ada_meal_scenario')

#name which we will add indices to
morsel_base_name = 'morsel'
def morsel_index_to_name(ind):
    return '{}{}'.format(morsel_base_name, ind)


delta_base_name = 'delta'
def delta_index_to_name(ind):
    return delta_base_name + str(ind)


def clear_all_morsels(env):
    # clear any existing morsels...
    # shouldn't need to do this but we can't trust that the trial before us actually cleaned up
    # this is hacky but what can you do
    for morsel_idx in range(100):
        prev_morsel = env.GetKinBody(
            morsel_index_to_name(morsel_idx))
        if prev_morsel is not None:
            env.RemoveKinBody(prev_morsel)

class DetectGoals(Future):
    def __init__(self, goal_tracker_cls, detection_topic, detection_msg, prev_result=None, config=None, status_cb=None):
        super(DetectGoals, self).__init__()
        status_cb("Detecting goals")
        self.env = config['env']
        self.robot = config['robot']
        clear_all_morsels(self.env)

        self._sub = rospy.Subscriber(detection_topic, detection_msg, self._recv_detection)
        self._tracker = goal_tracker_cls()

        # time out if no detections
        self._timer = rospy.Timer(rospy.Duration(5.), self._time_out, oneshot=True)

        # clean up after ourselves
        # shouldn't be necessary but gc is inconsistent so who knows how long the callback will keep running
        self.add_done_callback(lambda _: self._sub.unregister())
        self.add_done_callback(lambda _: self._timer.shutdown())


    def _recv_detection(self, msg):
        try:
            done = self._tracker.update(msg)
        except RuntimeError as e:
            self.set_exception(e)
        if done:
            # we got some goals!
            # make sure we don't timeout + finish at the same time
            with self.lock:
                if not self.done(): # this locks again, but it's an RLock so that
                                    # is fine. better to use the public api
                    self.set_result(self._tracker.get_goals(self.env, self.robot))

    def _time_out(self, evt):
        # make sure we don't double-call the timeout
        # make sure we don't timeout + finish at the same time
        with self.lock:
            if not self.done(): # this locks again, but it's an RLock so that
                                # is fine. better to use the public api
                # timed out
                # self.set_exception(RuntimeError('Timed out waiting for goals'))
                # nah, just return what we have...
                self.set_result(self._tracker.get_goals(self.env, self.robot))


@futurize(blocking=True)
def GenerateDummyMorsels(prev_result=None, config=None, status_cb=None, num_morsels=3):
    clear_all_morsels(config['env'])
    morsels = []
    for i in range(num_morsels):
        # Here we want to place the kinbody
        #  somewhere in the environment
        base_pos = [0.02, -0.02, 0.52]

        #add random noise
        rand_max_norm = 0.15
        base_pos[:2] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm

#             #switch to this if you want to test noise in world frame, not camera frame
#             camera_in_world = config['robot'].GetLink('Camera_Depth_Frame').GetTransform()
#             morsel_in_world = numpy.dot(camera_in_world, morsel_in_camera)
# #            morsel_in_world[0:2, 3] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm
#             #morsel_in_world[2,3] -= 0.17
#             morsel_in_camera = numpy.dot(numpy.linalg.inv(camera_in_world), morsel_in_world)
        morsels.append(create_kinbody_for_morsel(config['env'], config['robot'], base_pos))

    finalize_morsels(config['env'], config['robot'], morsels)
    return morsels




class MorselDetector(object):
    def __init__(self):
        #keep track of hypotheses for morsel locations
        self.morsel_pos_hypotheses = []
        self.morsel_pos_hypotheses_counts = []

        # todo: move this stuff to a config somewhere?
        #require this many consecutive detections to add morsels
        self.min_counts_required_addmorsels = 5
        #once that threshhold is reached for any morsel, require this many counts per morsel to add
        self.min_counts_required = 3
        #if less then this treshhold distance, count as consecutive
        self.distance_thresh_count = 0.02
        # filtering threshold
        self.dist_thresh_below_table = 0.
        self.dist_thresh_above_table = 0.1
        # fixed dist above table for projection
        self.dist_above_table = 0.03
       
    def update(self, msg):
        logger.debug('Received detection')
        obj =  json.loads(msg.data)
        pts_arr = obj['pts3d']
        morsel_positions = numpy.asarray(pts_arr)
        if morsel_positions is None or len(morsel_positions) == 0:
            return False

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

        #if any detection exceeds min count, we're done I guess
        return max(self.morsel_pos_hypotheses_counts) >= self.min_counts_required

    def get_goals(self, env, robot):
        # load some env info
        table = env.GetKinBody('table')
        table_aabb = table.ComputeAABB()
        top_of_table = table_aabb.pos()[2] + table_aabb.extents()[2]

        # load morsel object (to get extents)

        # take our current morsels and generate a list of goals
        # first filter for various things"
        #    - # detections >= min
        #    - morsel within thresh of table
        morsels = []
        for hypoth_pos, hypoth_pos_count in zip(self.morsel_pos_hypotheses, self.morsel_pos_hypotheses_counts):
            logger.info('checking morsel at pos ' + str(hypoth_pos))
            if hypoth_pos_count >= self.min_counts_required:
                # generate extents
                # hopefully we can do this without calling env.add, since we don't actually know if this is a real dxn yet
                # just using it to load the uri
                # if we need to call env.add(), make sure to lock env first! (with env)
                morsel = create_kinbody_for_morsel(env, robot, hypoth_pos)
                morsel_aabb = morsel.ComputeAABB()

                bottom_of_object = morsel_aabb.pos()[2] - morsel_aabb.extents()[2]
                dist_diff = (bottom_of_object - top_of_table)

                # filter the offsets
                if dist_diff >= self.dist_thresh_below_table and dist_diff <= self.dist_thresh_above_table:
                    # we have a detection! yay
                    # project it onto the table
                    morsel_transform = morsel.GetTransform()
                    morsel_transform[2, 3] -= dist_diff - self.dist_above_table
                    morsel.SetTransform(morsel_transform)

                    logger.info('adding morsel at pos ' + str(hypoth_pos))
                    morsels.append(morsel)

        # we have a full list of morsels, yay
        # now add them to the env
        finalize_morsels(env, robot, morsels)
        return morsels

def create_kinbody_for_morsel(env, robot, morsel_pos):
    morsel_in_camera = numpy.eye(4)
    morsel_in_camera[:3, 3] = morsel_pos

    camera_in_world = robot.GetLink(
        'Camera_Depth_Frame').GetTransform()
    morsel_in_world = numpy.dot(camera_in_world, morsel_in_camera)

    object_base_path = find_in_workspaces(
        search_dirs=['share'],
        project='ada_meal_scenario',
        path='data',
        first_match_only=True)[0]
    ball_path = os.path.join(
        object_base_path, 'objects', 'smallsphere.kinbody.xml')
    morsel = env.ReadKinBodyURI(ball_path) # we could cache this... but probably not worth it
    morsel.SetTransform(morsel_in_world)
    return morsel


def finalize_morsels(env, robot, morsels):
    for morsel_idx, morsel in enumerate(morsels):
        morsel.SetName(morsel_index_to_name(morsel_idx))
        morsel.Enable(False)
    with env:
        for morsel in morsels:
            env.Add(morsel)


DetectMorsels = partial(DetectGoals, goal_tracker_cls=MorselDetector,
                        detection_topic='/perception/morsel_detection', detection_msg=String)



    
