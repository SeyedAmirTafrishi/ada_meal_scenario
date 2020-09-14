
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
    def __init__(self, action_factories, prev_result=None, config={}, status_cb=lambda _: None):
        super(ActionSequence, self).__init__()
        self.current_action = None
        self.action_factories = deque(action_factories)
        self.config = config
        self.status_cb = status_cb if status_cb is not None else lambda _: None

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
            logger.debug('finished action {}'.format(action))
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
            logger.debug('running next action {}'.format(next_action_factory))
            self.current_action = next_action_factory(
                prev_result=result, config=self.config, status_cb=self.status_cb)
            self.current_action.add_done_callback(self._action_finished)
        except Exception as ex:
            # give full info in the console
            # need to do it here bc re-throwing it in a different thread chances the traceback
            traceback.print_exc()
            self.set_exception(ex)


class LoggedActionSequence(ActionSequence):
    """
    Overload method of ActionSequence to provide status updates to (1) rospy logging (2) ros topic (3) supplied cb (used for gui)
    """
    def __init__(self, action_factories, prev_result=None, config={}, status_cb=lambda _: None):
        self.pub = rospy.Publisher(config.get('status_topic', 'ada_tasks'), String, queue_size=10)
        self._status_cb = status_cb
        super(LoggedActionSequence, self).__init__(action_factories, prev_result, config, self.set_status)

    def set_status(self, status):
        self.pub.publish(status)
        rospy.loginfo(status)
        self._status_cb(status)


class ActionSequenceFactory:
    def __init__(self, action_factories=[]):
        self.action_factories = action_factories

    def then(self, factory):
        self.action_factories.append(factory)
        return self # for chaining
    
    def __call__(self, *args, **kwargs):
        return ActionSequence(self.action_factories, *args, **kwargs)

    def run(self, *args, **kwargs):
        return self.__call__(*args, **kwargs)




def defer_threaded(fn, args=(), kwargs={}):
    future = Future()
    def run(*args, **kwargs):
        try:
            future.set_result(fn(*args, **kwargs))
        except Exception as ex:
            future.set_exception(ex)
    future._handle = threading.Thread(target=run, args=args, kwargs=kwargs)
    future._handle.daemon = True  # not part of constructor until python3.3
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


def make_async_mapper(fn, itr):
    # collect the inputs, initialize storage for the outputs
    inputs = list(itr)
    res_obj = [ None ] * len(inputs)

    def make_func_runner(idx, inpt):
        def run(*args, **kwargs):
            future = defer_threaded(fn, inpt)
            def update_result(fut):
                res_obj[idx] = fut.result(0)
            future.add_done_callback(update_result)
            return future
        return run
    
    action_factories = [ make_func_runner(idx, inpt) for idx, inpt in enumerate(inputs) ]
    # make sure we actually return the result
    def return_result(*args, **kwargs):
        return NoOp(res_obj)
    action_factories += [return_result]
    return ActionSequenceFactory(action_factories=action_factories)
