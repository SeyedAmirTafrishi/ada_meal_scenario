import logging
import os
import pexpect
import rosgraph
import rospy
import signal
import subprocess
import sys
import time

try:
    import Tkinter as tk
    import tkMessageBox as tk_msg
except ImportError:
    import tkinter as tk # python3

from ada_meal_scenario.action_sequence import futurize

ROSBAG_RECORDER_CONFIG_NAME = 'rosbag'

class RosbagRecorder:
    def __init__(self, log_dir, topics):
        # check that all topics have publishers, warn if any aren't being published
        # to help catch typos in topic names
        # TODO(rma) make this check optional/configurable
        print('running on topics: {}'.format(topics))
        avail_topics = [ topic[0] for topic in rospy.get_published_topics() ] 
        for topic in topics:
            if topic not in avail_topics:
                rospy.logwarn('Requested rosbag logging of {} but it does not seem to be published. Did you spell the topic correctly?'.format(topic))

        self.filename = os.path.join(log_dir, 'trial_data') # rosbag exec adds .bag extension
        self.topics = topics
        self._proc = None

    def start(self):
        # run the rosbag collection in a different process
        # for (1) latency (2) ease of connecting to multiple topics
        # would be nice to make this in-process or at least in python (multiprocessing instead of subprocess)
        self._node_name = rosgraph.names.anonymous_name('rosbag_recorder')
        proc_args = 'rosbag record -O {filename} {topics} __name:={name}'.format(filename=self.filename, topics=' '.join(self.topics), name=self._node_name)
        rospy.logdebug('Starting rosbag process with command %s', proc_args)
        self._proc = pexpect.spawn(proc_args, logfile=sys.stdout)

        # wait for rosbag to be available
        # nb: this is terrible. melodic introduces a pub message that goes out when the recording starts
        try:
            self._proc.expect_exact('Recording', timeout=30.)  # this timeout is probably way too long but it's what i need for testing on docker...
        except pexpect.TIMEOUT:
            rospy.logerr('Failed to start rosbag recording!')
            self._proc.terminate(force=True)
            self._proc = None
            self._node_name = None

    def stop(self):
        # first try to stop it via ros
        try:
            subprocess.check_call(['rosnode', 'kill', self._node_name])
            self._proc.expect_exact(['Shutting down', pexpect.EOF, pexpect.TIMEOUT], timeout=0.5)
        except subprocess.CalledProcessError as e:
            rospy.logwarn('Failed to shut down rosbag process: %s', str(e))

        if self._proc.isalive():
            rospy.logwarn('Failed to shut down rosbag process, terminating...')
            if not self._proc.terminate(force=True):
                rospy.logerr('Failed to terminate rosbag process!')

        self._proc = None
        self._node_name = None
    
    def __del__(self):
        # clean up extra process if necessasry
        if self._proc is not None:
            self.stop()


class RosbagRecorderConfigFrame(tk.LabelFrame, object):
    def __init__(self, parent, initial_config={}):
        super(RosbagRecorderConfigFrame, self).__init__(parent, text='ROS Bag')
        initial_config = initial_config.get(ROSBAG_RECORDER_CONFIG_NAME, {})

        self.enabled_var = tk.BooleanVar(value=initial_config.get('enabled', False))
        self.enabled_checkbox = tk.Checkbutton(self, variable=self.enabled_var, text="Enable ROS Bag recording")
        self.enabled_checkbox.grid(row=0, column=0, columnspan=2, sticky=tk.N+tk.W)

        self.topics_label = tk.Label(self, text='Topics to record')
        self.topics_label.grid(row=1, column=0, columnspan=2, sticky=tk.N+tk.W)
        self.topics_listbox_frame = tk.Frame(self)
        self.topics_listbox_scroll = tk.Scrollbar(self.topics_listbox_frame, orient=tk.VERTICAL)
        self.topics_listbox = tk.Listbox(self.topics_listbox_frame, selectmode=tk.MULTIPLE, exportselection=0, yscrollcommand=self.topics_listbox_scroll.set)
        self.topics_listbox_scroll.configure(command=self.topics_listbox.yview)
        self.topics_listbox_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.topics_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)
        self.topics_listbox_frame.grid(row=2, column=0, columnspan=2, sticky=tk.N+tk.E+tk.S+tk.W)

        # initialize the topics
        cur_topics = set(topic for topic, _ in rospy.get_published_topics())
        sel_topics = initial_config.get('topics', {})
        cur_topics.update(sel_topics)

        for topic in sorted(cur_topics):
            self.topics_listbox.insert(tk.END, topic)
            if topic in sel_topics:
                self.topics_listbox.selection_set(tk.END)

        self.topics_add_var = tk.StringVar()
        self.topics_add_entry = tk.Entry(self, textvariable=self.topics_add_var)
        self.topics_add_entry.bind("<Return>", self._add_topic)
        self.topics_add_button = tk.Button(self, text='Add', command=self._add_topic)
        self.topics_add_entry.grid(row=3, column=0, sticky=tk.N+tk.E+tk.W)
        self.topics_add_button.grid(row=3, column=1, sticky=tk.W)

        self.rowconfigure(2, weight=1)
        self.columnconfigure(0, weight=1)

    def _add_topic(self, _=None):
        topic = self.topics_add_var.get()

        if topic in self.topics_listbox.get(0, tk.END):
            # already exists
            return
        # validate
        if not rosgraph.names.is_legal_name(topic):
            tk_msg.showerror(message='Topic {} is not a valid ROS topic'.format(topic))
            return

        # we look good
        # insert it at the end (todo maybe, sort them alphabetically or something?)
        self.topics_listbox.insert(tk.END, topic)
        self.topics_listbox.selection_set(tk.END)
        self.topics_listbox.see(tk.END)

    def get_config(self):
        return { ROSBAG_RECORDER_CONFIG_NAME: {
            'enabled': self.enabled_var.get(),
            'topics': [ self.topics_listbox.get(int(i)) for i in self.topics_listbox.curselection() ]
        }}

    def set_state(self, state):
        self.enabled_checkbox.configure(state=state)
        self.topics_listbox.configure(state=state)
        self.topics_add_entry.configure(state=state)
        self.topics_add_button.configure(state=state)


def get_rosbag_recorder(log_dir, config):
    if ROSBAG_RECORDER_CONFIG_NAME in config and config[ROSBAG_RECORDER_CONFIG_NAME]['enabled']:
        return RosbagRecorder(log_dir, config[ROSBAG_RECORDER_CONFIG_NAME]['topics'])
    else:
        return None

