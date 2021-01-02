
import rospy
import tf2_ros
import tf.transformations

import geometry_msgs.msg
import std_msgs.msg

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk # python3


GOAL_TRANSFORM_PUBLISHER_CONFIG_NAME = 'goal_transform_publisher'


class GoalTransformPublisherConfigFrame(tk.Frame, object):
    def __init__(self, parent, initial_config={}):
        super(GoalTransformPublisherConfigFrame, self).__init__(parent)

        initial_config = initial_config.get(GOAL_TRANSFORM_PUBLISHER_CONFIG_NAME, {})

        self._pub_val = tk.IntVar()
        self._pub_val.set(initial_config.get("publish", 0))
        self._pub_check = tk.Checkbutton(self, variable=self._pub_val, text="Publish goal transforms")
        self._pub_check.pack(anchor="nw")
    
    def get_config(self):
        return { GOAL_TRANSFORM_PUBLISHER_CONFIG_NAME: {
            "publish": bool(self._pub_val.get())
        }}
    
    def set_state(self, state):
        self._pub_check.configure(state=state)

def build_tf_msg(obj):
    return geometry_msgs.msg.TransformStamped(
        header=std_msgs.msg.Header(
            frame_id="map"
        ),
        child_frame_id=obj.name,
        transform=geometry_msgs.msg.Transform(
            translation=geometry_msgs.msg.Vector3(
                *tf.transformations.translation_from_matrix(obj.pose).tolist()
            ),
            rotation=geometry_msgs.msg.Quaternion(
                *tf.transformations.quaternion_from_matrix(obj.pose).tolist()
            )
        )
    )

class GoalTransformPublisher:
    RATE = rospy.Duration(1.)
    def __init__(self, objs):
        self._tf_pub = tf2_ros.TransformBroadcaster()
        self._msgs = [ build_tf_msg(o) for o in objs ]
    
    def start(self):
        self._timer = rospy.Timer(GoalTransformPublisher.RATE, self._timer_callback)

    def stop(self):
        self._timer.shutdown()

    def _timer_callback(self, msg):
        # update all the stamps
        for m in self._msgs:
            m.header.stamp = msg.current_expected + rospy.Duration(0.5)  # static, so forward-date
        self._tf_pub.sendTransform(self._msgs)
    
def get_goal_transform_publisher(goals):
    if len(goals) > 0:
        return GoalTransformPublisher(goals)
    else:
        return None
