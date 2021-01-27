# gui for interacting with the ada_meal_scenario
# used to select the method and user input device

import os
import Tkinter
import tkFont, tkMessageBox
from functools import partial
import traceback
import yaml
from collections import deque
import rospkg

from adapy.futures import TimeoutError, CancelledError
from ada_meal_scenario.assistance.assistance_config import AssistanceConfigFrame
from ada_meal_scenario.loggers.loggers import LoggingOptions



default_bg_color = None


def _load_initial_config(fn):
    if fn is not None and os.path.isfile(fn):
        with open(fn, 'r') as f:
            return yaml.load(f)
    else:
        return {}

class GuiHandler(object):
    def __init__(self, start_trial_callback, quit_callback, initial_config_file=None):
        self.config_file = os.path.abspath(initial_config_file) if initial_config_file is not None else None
        self._initial_config = _load_initial_config(initial_config_file)
        
        self.master = Tkinter.Tk()

        self.start_trial_callback = start_trial_callback
        self.quit_callback = quit_callback

        self.default_font = tkFont.nametofont("TkDefaultFont")
        self.default_font.configure(size=10)
        self.master.option_add("*Font", self.default_font)

        global default_bg_color
        default_bg_color = self.master.cget("bg")


        sticky = Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S
        self.config_button_frame = Tkinter.Frame(self.master)
        self.config_button_frame.grid(row=0, column=0, sticky='nsew', ipadx=3)

        self.config_frame = Tkinter.Frame(self.master)
        self.config_frame.grid(row=0, column=1, sticky='nsew', padx=2, pady=2)

        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(1, weight=1)
        
        self.config_frames = []
        self.config_selector_buttons = []
        self.active_frame = None


        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(row=1, column=0, columnspan=2, sticky='esw', padx=2, pady=2)

        self.start_button = Tkinter.Button(self.start_frame,
                                           text="Start Next Trial",
                                           command=self._start_button_callback)
        self.start_button.grid(row=0, column=0, sticky=Tkinter.W+Tkinter.E)

        self.cancel_button = Tkinter.Button(
            self.start_frame, text="Cancel trial", command=self._cancel_button_callback)
        self.cancel_button.grid(row=0, column=1, sticky=Tkinter.W+Tkinter.E)

        self.save_config_button = Tkinter.Button(self.start_frame, text='Save config', command=self._save_config)
        self.save_config_button.grid(row=0, column=2, sticky=Tkinter.N+Tkinter.W)

        self.quit_button = Tkinter.Button(
            self.start_frame, text="Quit", command=self._quit_button_callback)
        self.quit_button.grid(row=0, column=3, sticky='ne', padx=5)

        # add a status bar
        self.status_var = Tkinter.StringVar(value='Ready')
        self.status_bar = Tkinter.Label(
            self.master, textvariable=self.status_var, bd=1, relief=Tkinter.SUNKEN, anchor=Tkinter.W)
        self.status_bar.grid(row=2, column=0, columnspan=4, sticky=Tkinter.S+Tkinter.E+Tkinter.W)
        self._status_queue = deque()  # thread-safe queue for passing status change requests

        # configure enabled/disabled for waiting for a trial
        self.trial = None
        self.set_waiting_for_trial()

    def add_config_frame(self, fn, name, button_sticky='new'):
        if self.trial is not None:
            raise RuntimeError('Cannot add config frame while trial is running!')
        frame = fn(self.config_frame, self._initial_config)
        self.config_frames.append(frame)
        self.set_waiting_for_trial()

        select_button = Tkinter.Button(self.config_button_frame, text=name, command=lambda: self._highlight_config(frame))
        select_button.grid(column=0, sticky=button_sticky)
        self.config_selector_buttons.append(select_button)

        if self.active_frame is None:
            self._highlight_config(frame)

        return frame

    def _highlight_config(self, frame):
        if self.active_frame is frame:
            return
        if self.active_frame is not None:
            # it's technically faster to store the index separately than to search for it each time
            # but like, len(active_frames) is small so it really doesn't matter
            prev_btn_idx = self.config_frames.index(self.active_frame)
            self.config_selector_buttons[prev_btn_idx].configure(relief=Tkinter.RAISED)
            self.active_frame.pack_forget()
        frame.pack(fill=Tkinter.BOTH, expand=True)
        self.active_frame = frame
        btn_idx = self.config_frames.index(frame)
        self.config_selector_buttons[btn_idx].configure(relief=Tkinter.SUNKEN)

    def run_once(self):
        self.master.update()
        # check if our trial is done
        # we have to do this by polling bc tk doesn't let us create callbacks from outside threads :(
        if self.trial is not None and self.trial.done():
            self.set_trial_finished()

        # see if we need to update the status
        # we have to poll for status changes since tk isn't thread-safe
        # and wrap it in a loop to handle multiple status updates in the same frame
        new_status = None
        while True:
            try:
                new_status = self._status_queue.popleft()
            except IndexError:
                break
        if new_status is not None:
            self.status_var.set(new_status)

    def set_status(self, status):
        self._status_queue.append(status)

    def _set_config_frame_state(self, state):
        for frame in self.config_frames:
            frame.set_state(state)

    def set_trial_running(self):
        # disable the buttons corresponding to a running trial
        self._set_config_frame_state(Tkinter.DISABLED)

        self.start_button.configure(state=Tkinter.DISABLED)
        self.quit_button.configure(state=Tkinter.DISABLED)
        self.status_var.set('Trial running')

        # enable the cancel button
        self.cancel_button.configure(state=Tkinter.NORMAL)

    def set_waiting_for_trial(self):
        # enable buttons related to configuring a trial
        self._set_config_frame_state(Tkinter.NORMAL)

        self.start_button.configure(state=Tkinter.NORMAL)
        self.quit_button.configure(state=Tkinter.NORMAL)

        # disable the cancel button
        self.cancel_button.configure(state=Tkinter.DISABLED)

    def set_trial_finished(self):
        # check how we ended
        try:
            res = self.trial.result(0)
            self.status_var.set('Trial completed successfully')
        except CancelledError:
            self.status_var.set('Trial cancelled')
        except TimeoutError:
            # critical failure
            assert False, "Trial ended but timed out accessing info!"
        except Exception as ex:
            # notify of the error
            tkMessageBox.showerror(title="Trial error", message=str(ex))
            self.status_var.set('Trial error: {}'.format(str(ex)))

        # clear the existing trial
        self.trial = None
        # and re-enable relevant buttons
        self.set_waiting_for_trial()


    def _start_button_callback(self, trial_fn=None):
        # get the config
        try:
            cfg = self.get_config()
        except ValueError as e:
            tkMessageBox.showerror(message=str(e))
            return

        # update enabled/disabled corresponding to running a trial
        self.set_trial_running()

        # call the callback with the current config
        if trial_fn is None:
            trial_fn = self.start_trial_callback
        self.trial = trial_fn(config=cfg, status_cb=self.set_status)

    def _cancel_button_callback(self):
        # disable the cancel button to remove duplicate calls
        self.cancel_button.configure(state=Tkinter.DISABLED)
        # request that the trial be canceled
        if self.trial is not None: # handle cancel after trial is already terminating
            self.status_var.set('Trial being cancelled...')
            self.trial.cancel()
        # when the cancel is successful, the trial will call its finished_callback
        # which will reset the gui

    def _save_config(self):
        if self.config_file is None:
            # prompt for it
            self.config_file = tkFileDialog.asksaveasfilename(title='Select location to save config file', defaultextension='.yaml')
        if self.config_file is not None:
            with open(self.config_file, 'w') as f:
                yaml.dump(self.get_config(), f)
            self.status_var.set('Config file saved to {}'.format(self.config_file))

    def _quit_button_callback(self):
        self.quit_callback()

    def get_config(self):
        to_ret = dict()
        for frame in self.config_frames:
            to_ret.update(frame.get_config())
        return to_ret

def build_ada_meal_scenario_gui_handler(*args, **kwargs):
    gui_handler = GuiHandler(*args, **kwargs)
    gui_handler.add_config_frame(AssistanceConfigFrame, "Assistance")
    gui_handler.add_config_frame(LoggingOptions, "Logging")
    
    return gui_handler


