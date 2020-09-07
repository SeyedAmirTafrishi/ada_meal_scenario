# gui for interacting with the ada_meal_scenario
# used to select the method and user input device

import Tkinter
import tkFont, tkFileDialog, tkMessageBox
from functools import partial
from adapy.futures import TimeoutError, CancelledError

import os
import rospkg

from ada_teleoperation import DataRecordingUtils


default_bg_color = None


# Basic tooltip
# From https://www.daniweb.com/programming/software-development/code/484591/a-tooltip-class-for-tkinter
class ToolTip(object):
    '''
    create a tooltip for a given widget
    '''

    def __init__(self, widget, text='widget info'):
        self.widget = widget
        self.text = text
        self.widget.bind("<Enter>", self.enter)
        self.widget.bind("<Leave>", self.close)

    def enter(self, event=None):
        x = y = 0
        x, y, cx, cy = self.widget.bbox("insert")
        x += self.widget.winfo_rootx() + 25
        y += self.widget.winfo_rooty() + 20
        # creates a toplevel window
        self.tw = Tkinter.Toplevel(self.widget)
        # Leaves only the label and removes the app window
        self.tw.wm_overrideredirect(True)
        self.tw.wm_geometry("+%d+%d" % (x, y))
        label = Tkinter.Label(self.tw, text=self.text, justify='left',
                         background='yellow', relief='solid', borderwidth=1,
                         font=("times", "8", "normal"))
        label.pack(ipadx=1)

    def close(self, event=None):
        if self.tw:
            self.tw.destroy()


# a is the robot action
# u is the user action
def transition(a, u, gamma):
    return a + u

def transition_1gamma(a, u, gamma):
    return gamma*a + (1-gamma)*u

def transition_2gamma(a, u, gamma):
    return 2*gamma*a + (2 - 2*gamma)*u


class OptionSelector(Tkinter.Frame, object):
    __BUTTON_CONFIG__ = {
        'indicatoron': False,
        'selectcolor': "#cc0000"
    }

    def __init__(self, parent, title, option_names, option_values, default_selection=None):
        super(OptionSelector, self).__init__(parent)

        label_font = tkFont.nametofont("TkDefaultFont").copy()
        label_font.configure(weight='bold')

        self.label = Tkinter.Label(
            self, text=title, font=label_font)
        self.label.grid(row=0, sticky=Tkinter.E+Tkinter.W)

        self.option_values = option_values
        self.variable = Tkinter.IntVar()

        self.buttons = []
        for idx, text in enumerate(option_names):
            button = Tkinter.Radiobutton(self, text=text, variable=self.variable, value=idx,
                                         **OptionSelector.__BUTTON_CONFIG__)
            button.grid(row=1+idx, column=0, sticky=Tkinter.E+Tkinter.W)
            self.buttons.append(button)

        if default_selection is not None:
            self.buttons[default_selection].select()

    def get_value(self):
        return self.option_values[self.variable.get()]

    def configure(self, **kwargs):
        for button in self.buttons:
            button.configure(**kwargs)


class LoggingOptions(Tkinter.Frame, object):
    __USER_ID_INVALID_BG__ = "#cc0000"
    def __init__(self, parent):
        super(LoggingOptions, self).__init__(parent)

        label_font = tkFont.nametofont("TkDefaultFont").copy()
        label_font.configure(weight='bold')

        self.label = Tkinter.Label(
            self, text="Logging Options", font=label_font)
        self.label.grid(row=0, sticky=Tkinter.E+Tkinter.W)

        default_log_dir = os.path.join(rospkg.RosPack().get_path(
            'ada_meal_scenario'), 'trajectory_data')
        
        # choose top dir for logging
        self.data_root_var = Tkinter.StringVar(value=default_log_dir)
        self.data_root_label = Tkinter.Label(
            self, textvariable=self.data_root_var)
        print('var: {}'.format(self.data_root_var.get()))
        self.data_root_label.grid(row=1, column=0, sticky=Tkinter.E+Tkinter.W)
        self.data_root_button = Tkinter.Button(self, text='Select data directory', command=self._set_data_root)
        self.data_root_button.grid(row=1, column=1, sticky=Tkinter.W)

        # choose user id
        self.user_id_var = Tkinter.StringVar()
        self.update_next_user_id()
        self.user_id_var.trace("w", self._validate_user_id)
        self.user_id_entry = Tkinter.Entry(
            self, textvariable=self.user_id_var)
        self.user_id_entry.grid(row=2, column=0, sticky=Tkinter.E+Tkinter.W)
        self.user_id_label = Tkinter.Label(self, text='User ID')
        self.user_id_label.grid(row=2, column=1, sticky=Tkinter.W)

        self.user_id_orig_bg = self.user_id_entry.cget("bg")

        # additional logging options
        self.pupil_labs_recording_var = Tkinter.BooleanVar()
        self.pupil_labs_recording = Tkinter.Checkbutton(
            self, text="Pupil Labs recording", variable=self.pupil_labs_recording_var)
        self.pupil_labs_recording.grid(row=3, column=0, sticky=Tkinter.E+Tkinter.W)

        self.zed_remote_recording_var = Tkinter.BooleanVar()
        self.zed_remote_recording = Tkinter.Checkbutton(
            self, text='ZED Remote recording', variable=self.zed_remote_recording_var)
        self.zed_remote_recording.grid(row=3, column=1, sticky=Tkinter.E+Tkinter.W)
        self.zed_remote_recording_avail = LoggingOptions.check_zed_remote_available()
        if not self.zed_remote_recording_avail:
            self.zed_remote_recording.configure(state=Tkinter.DISABLED)
            self.zed_remote_recording_tooltip = ToolTip(
                self.zed_remote_recording, 'No ZED package found; ZED videos will NOT be recorded. Install the zed_ros_recording package to fix this error.')



    @staticmethod
    def check_zed_remote_available():
        try:
            from zed_recorder.srv import ZedRecord, ZedRecordRequest
            return True
        except ImportError:
            return False


    def _set_data_root(self):
        data_root = tkFileDialog.askdirectory(initialdir=self.data_root_var.get(), title='Choose root directory for logging')
        if data_root is not None:
            self.data_root_var.set(data_root)

    def _get_data_dir(self):
        return DataRecordingUtils.get_filename(
            self.data_root_var.get(), DataRecordingUtils.user_folder_base_default, self.user_id_var.get(), '')
    
    def _validate_user_id(self, *_):
        data_dir = self._get_data_dir()
        if os.path.exists(data_dir):
            self.user_id_entry.config(bg=LoggingOptions.__USER_ID_INVALID_BG__)
        else:
            self.user_id_entry.config(bg=self.user_id_orig_bg)

    def get_config(self):
        data_dir = self._get_data_dir()
        if os.path.exists(data_dir):
            raise ValueError("Directory exists: {}".format(data_dir))
        return {
            'data_dir': data_dir,
            'record_pupil': self.pupil_labs_recording_var.get(),
            'record_zed_remote': self.zed_remote_recording_var.get()
        }

    def set_state(self, state):
        self.data_root_button.configure(state=state)
        self.user_id_entry.configure(state=state)
        self.pupil_labs_recording.configure(state=state)

        # don't reset the state if we have no zed
        if self.zed_remote_recording_avail:
            self.zed_remote_recording.configure(state=state)

    def update_next_user_id(self):
        # when we've finished a trial, we need to advance the user id
        default_user_id, _ = DataRecordingUtils.get_next_available_user_ind(
            self.data_root_var.get(), make_dir=False)
        self.user_id_var.set(default_user_id)


class GuiHandler(object):
    def __init__(self, base_config, start_trial_callback, quit_callback):
        self.master = Tkinter.Tk()

        self.base_config = base_config
        self.start_trial_callback = start_trial_callback
        self.quit_callback = quit_callback

        self.default_font = tkFont.nametofont("TkDefaultFont")
        self.default_font.configure(size=10)
        self.master.option_add("*Font", self.default_font)

        global default_bg_color
        default_bg_color = self.master.cget("bg")

        #Tkinter.Grid.rowconfigure(self.master, 0, weight=10)
        #Tkinter.Grid.columnconfigure(self.master, 0, weight=10)
        sticky = Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S

        self.method_selector = OptionSelector(self.master, title="Method: \n",
                                              option_names=[
                                                  'Direct Teleop', 'Shared Auton Lvl 1', 'Shared Auton Lvl 2', 'Full Auton User Goal', 'Full Auton Random Goal'],
                                              option_values=[['direct', 0.0], ['shared_auton_1', 0.33], [
                                                  'shared_auton_2', 0.66], ['shared_auton_3', 1.0], ['autonomous', None]],
                                              default_selection=4)
        self.method_selector.grid(sticky=sticky)

        self.device_selector = OptionSelector(self.master, title="UI Device\n", option_names=['Mouse', 'Kinova USB'], 
            option_values=['mouse', 'kinova'], default_selection=1)
        self.device_selector.grid(row=0, column=1, padx=2, pady=2, sticky=sticky)

        self.transition_function_selector = OptionSelector(
            self.master, title="Transition Function: \n", option_names=['a+u', 'gamma*a + (1-gamma)*u', '2*gamma*a + (2-2*gamma)*u'], 
            option_values=[transition, transition_1gamma, transition_2gamma], default_selection=0)
        self.transition_function_selector.grid(
            row=0, column=2, padx=2, pady=2, sticky=sticky)

        self.prediction_selector = OptionSelector(
            self.master, title="Prediction Method: \n", option_names=['Policy', 'Gaze', 'Merged'],
            option_values=['policy', 'gaze', 'merged'], default_selection=0)
        self.prediction_selector.grid(
            row=0, column=3, padx=2, pady=2, sticky=sticky)

        self.logging_options = LoggingOptions(self.master)
        self.logging_options.grid(
            row=1, column=1, padx=2, pady=2, columnspan=3, sticky=sticky)


        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(row=1, column=0, sticky=sticky, padx=2, pady=2)

        self.start_button = Tkinter.Button(self.start_frame,
                                           text="Start Next Trial",
                                           command=self._start_button_callback)
        self.start_button.grid(row=0, column=0, sticky=Tkinter.W+Tkinter.E)

        self.cancel_button = Tkinter.Button(
            self.start_frame, text="Cancel trial", command=self._cancel_button_callback)
        self.cancel_button.grid(row=1, column=0, sticky=Tkinter.W+Tkinter.E)

        self.quit_button = Tkinter.Button(
            self.start_frame, text="Quit", command=self._quit_button_callback)
        self.quit_button.grid(row=2, column=0, sticky=Tkinter.W+Tkinter.E)

        # add a status bar
        self.status_var = Tkinter.StringVar(value='Ready')
        self.status_bar = Tkinter.Label(
            self.master, textvariable=self.status_var, bd=1, relief=Tkinter.SUNKEN, anchor=Tkinter.W)
        self.status_bar.grid(row=2, column=0, columnspan=4, sticky=Tkinter.S+Tkinter.E+Tkinter.W)

        # configure enabled/disabled for waiting for a trial
        self.trial = None
        self.set_waiting_for_trial()


    def run_once(self):
        self.master.update()
        # check if our trial is done
        # we have to do this by polling bc tk doesn't let us create callbacks from outside threads :(
        if self.trial is not None and self.trial.done():
            self.set_trial_finished()

    def set_trial_running(self):
        # disable the buttons corresponding to a running trial
        self.method_selector.configure(state=Tkinter.DISABLED)
        self.device_selector.configure(state=Tkinter.DISABLED)
        self.transition_function_selector.configure(state=Tkinter.DISABLED)
        self.prediction_selector.configure(state=Tkinter.DISABLED)
        self.logging_options.set_state(Tkinter.DISABLED)

        self.start_button.configure(state=Tkinter.DISABLED)
        self.quit_button.configure(state=Tkinter.DISABLED)
        self.status_var.set('Trial running')

        # enable the cancel button
        self.cancel_button.configure(state=Tkinter.NORMAL)

    def set_waiting_for_trial(self):
        # enable buttons related to configuring a trial
        self.method_selector.configure(state=Tkinter.NORMAL)
        self.device_selector.configure(state=Tkinter.NORMAL)
        self.transition_function_selector.configure(state=Tkinter.NORMAL)
        self.prediction_selector.configure(state=Tkinter.NORMAL)
        self.logging_options.set_state(Tkinter.NORMAL)
        self.logging_options.update_next_user_id()

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
        except RuntimeError as ex:
            # notify of the error
            tkMessageBox.showerror(title="Trial error", message=str(ex))
            self.status_var.set('Trial error: {}'.format(str(ex)))

        # clear the existing trial
        self.trial = None
        # and re-enable relevant buttons
        self.set_waiting_for_trial()


    def _start_button_callback(self):
        # get the config
        try:
            cfg = self.base_config.copy()
            cfg.update(self.get_selected_options())
        except ValueError as e:
            tkMessageBox.showerror(message=str(e))
            return

        # update enabled/disabled corresponding to running a trial
        self.set_trial_running()

        # call the callback with the current config
        self.trial = self.start_trial_callback(config=cfg)

    def _cancel_button_callback(self):
        # disable the cancel button to remove duplicate calls
        self.cancel_button.configure(state=Tkinter.DISABLED)
        # request that the trial be canceled
        if self.trial is not None: # handle cancel after trial is already terminating
            self.status_var.set('Trial being cancelled...')
            self.trial.cancel()
        # when the cancel is successful, the trial will call its finished_callback
        # which will reset the gui


    def _quit_button_callback(self):
        self.quit_callback()

    def get_selected_options(self):
        to_ret = dict()
        to_ret['method'] = self.method_selector.get_value()[0]
        to_ret['ui_device'] = self.device_selector.get_value()
        to_ret['transition_function'] = partial(
            self.transition_function_selector.get_value(), gamma=self.method_selector.get_value()[1])
        to_ret['prediction_option'] = self.prediction_selector.get_value()
        to_ret['logging'] = self.logging_options.get_config()
        return to_ret


