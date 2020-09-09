
from adapy.futures import Future, TimeoutError, CancelledError
from std_msgs.msg import String
from collections import deque
import rospy
import functools
import threading
import traceback

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
        # make sure the next action we do is a cancel action
        def cancel_factory(*args, **kwargs):
            future = Future()
            future.set_cancelled()
            return future
        self.action_factories.appendleft(cancel_factory) # deque is thread-safe

        try:
            self.current_action.cancel()
        except NotImplementedError:
            # we can't cancel this operation!
            # so make sure it's the last one called
            logger.warning(
                'Unable to cancel current action; waiting till it terminates to cancel')


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
        except Exception as ex:
            # give full info in the console
            # need to do it here bc re-throwing it in a different thread chances the traceback
            traceback.print_exc()
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
        except Exception as ex:
            # give full info in the console
            # need to do it here bc re-throwing it in a different thread chances the traceback
            traceback.print_exc()
            self.set_exception(ex)


def defer_threaded(fn, args, kwargs):
    future = Future()

    def run(*args, **kwargs):
        try:
            future.set_result(fn(*args, **kwargs))
        except Exception as ex:
            future.set_exception(ex)
    future._handle = threading.Thread(target=run, args=args, kwargs=kwargs)
    future._handle.daemon = True
    future._handle.start()
    return future


def futurize_threaded(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs): # rewrap so we can use functools.wraps for docstrings etc
        return defer_threaded(fn, args, kwargs)
    return wrapper


def defer_blocking(fn, args=(), kwargs={}):
    future = Future()
    try:
        future.set_result(fn(*args, **kwargs))
    except Exception as ex:
        future.set_exception(ex)
    return future

def futurize_blocking(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        return defer_blocking(fn, args, kwargs)
    return wrapper

def defer(blocking, target, args=(), kwargs={}):
    if blocking:
        return defer_blocking(target, args, kwargs)
    else:
        return defer_threaded(target, args, kwargs)

def futurize(blocking=False):
    if blocking:
        return futurize_blocking
    else:
        return futurize_threaded


class NoOp(Future):
    def __init__(self, result=None, *args, **kwargs):
        super(NoOp, self).__init__()
        self.set_result(result)


