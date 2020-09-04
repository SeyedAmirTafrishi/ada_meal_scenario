
from adapy.futures import Future, TimeoutError, CancelledError
from std_msgs.msg import String
from collections import deque
import rospy

class ActionSequence(Future):
    def __init__(self, action_factories=[], config=None):
        super(ActionSequence, self).__init__()
        self.current_action = None
        self.action_factories = deque(action_factories)
        self.config = config

        # start the first action
        self._start_next_action()

    def cancel(self):
        self.current_action.cancel()

    def _action_finished(self, action):
        # nonsense check
        assert action is self.current_action
        # make sure we ended the last one successfully
        try:
            result = action.result(0)
            # start the next action
            self._start_next_action(result)
        except TimeoutError:
            assert False, "Trial ended but timed out accessing info!"
        except CancelledError:
            self.set_cancelled()
        except RuntimeError as ex:
            self.set_exception(ex)
    
    def _start_next_action(self, result=None):
        # get the next action to run
        try:
            next_action_factory = self.action_factories.popleft()
        except IndexError:
            # no more actions
            self.set_result(result)
            return

        # run it   
        try:
            self.current_action = next_action_factory(
                prev_result=result, config=self.config)
            self.current_action.add_done_callback(self._action_finished)
        except RuntimeError as ex:
            self.set_exception(ex)


