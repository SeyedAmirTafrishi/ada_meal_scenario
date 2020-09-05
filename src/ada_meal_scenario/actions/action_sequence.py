
from adapy.futures import Future, TimeoutError, CancelledError
from std_msgs.msg import String
from collections import deque
import rospy
import functools
import threading

import logging
project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

class ActionSequence(Future):
    def __init__(self, action_factories=[], config=None):
        super(ActionSequence, self).__init__()
        self.current_action = None
        self.action_factories = deque(action_factories)
        self.config = config

        # start the first action
        self._start_next_action()

    def cancel(self):
        try:
            self.current_action.cancel()
        except NotImplementedError:
            # we can't cancel this operation!
            # so make sure it's the last one called
            logger.warning(
                'Unable to cancel current action; waiting till it terminates to cancel')

            def cancel_factory(*args, **kwargs):
                future = Future()
                future.set_cancelled()
                return future

            # if we're on the last action, just let it finish
            # otherwise stick a cancel action as the next action
            if  len(self.action_factories) > 0:
                # OK not to make this thread-safe with len() above
                # Worst case is we start the last action between the if statement above and this append function below
                # in that case, even though the trial technically finished, we mark it as cancelled
                # but if the user requested cancellation, seems like not a problem
                self.action_factories.appendleft(cancel_factory)

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

def make_future(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        future = Future()
        def run(*args, **kwargs):
            try:
                future.set_result(fn(*args, **kwargs))
            except RuntimeError as ex:
                future.set_exception(ex)
        future._handle = threading.Thread(target=run, args=args, kwargs=kwargs)
        future._handle.daemon = True
        future._handle.start()
        return future
    return wrapper

class NoOp(Future):
    def __init__(self, result=None, *args, **kwargs):
        super(NoOp, self).__init__()
        self.set_result(result)


