# gui for interacting with the ada_meal_scenario
# used to select the method and user input device

import Tkinter
import tkFont
from functools import partial

from threading import Lock
import multiprocessing
from Queue import Empty
from multiprocessing import Queue

default_bg_color = None

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

        label_font = tkFont.nametofont("TkDefaultFont")
        label_font.configure(weight='bold', size=14)

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


class GuiHandler(object):
    def __init__(self, get_gui_state_event, trial_starting_event, return_queue):
        self.master = Tkinter.Tk()
        self.record_next_trial = False
        self.start_next_trial = False
        self.quit = False

        self.get_gui_state_event = get_gui_state_event
        self.trial_starting_event = trial_starting_event
        self.return_queue = return_queue

        self.default_font = tkFont.nametofont("TkDefaultFont")
        self.default_font.configure(size=14)
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
        self.device_selector.grid(row=0, column=1, sticky=sticky)

        self.transition_function_selector = OptionSelector(
            self.master, title="Transition Function: \n", option_names=['a+u', 'gamma*a + (1-gamma)*u', '2*gamma*a + (2-2*gamma)*u'], 
            option_values=[transition, transition_1gamma, transition_2gamma], default_selection=0)
        self.transition_function_selector.grid(row=0, column=2, sticky=sticky)

        self.prediction_selector = OptionSelector(
            self.master, title="Prediction Method: \n", option_names=['Policy', 'Gaze', 'Merged'],
            option_values=['policy', 'gaze', 'merged'], default_selection=0)
        self.prediction_selector.grid(row=0, column=3, sticky=sticky)


        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(row=1, column=2, sticky=sticky)

        self.record_button = Tkinter.Button(self.start_frame,
                                            text="Record Next Trial",
                                            command=self.record_button_callback)
        self.record_button.grid(sticky=Tkinter.W+Tkinter.E)

        self.start_button = Tkinter.Button(self.start_frame,
                                           text="Start Next Trial",
                                           command=self.start_button_callback)
        self.start_button.grid(sticky=Tkinter.W+Tkinter.E)

        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(
            row=1, column=0, sticky=Tkinter.W+Tkinter.E+Tkinter.S)

        self.quit_button = Tkinter.Button(
            self.start_frame, text="Quit", command=self.quit_button_callback)
        self.quit_button.grid(sticky=Tkinter.W+Tkinter.E)


    def mainloop(self):
        # self.master.mainloop()
        import time
        while True:
            self.master.update_idletasks()
            self.master.update()

            if self.get_gui_state_event.is_set():
                self.add_return_to_queue()
                self.get_gui_state_event.clear()

            if self.trial_starting_event.is_set():
                self.start_next_trial = False
                configure_button_not_selected(self.start_button)
                self.trial_starting_event.clear()

            time.sleep(0.01)


    def start_button_callback(self):
        self.start_next_trial = toggle_trial_button_callback(
            self.start_button, self.start_next_trial)
        # self.add_return_to_queue()

    def record_button_callback(self):
        self.record_next_trial = toggle_trial_button_callback(
            self.record_button, self.record_next_trial)

    def quit_button_callback(self):
        self.quit = toggle_trial_button_callback(self.quit_button, self.quit)
        # self.add_return_to_queue()

    def add_return_to_queue(self):
        curr_selected = self.get_selected_options()
        # while not self.return_queue.empty():
        #    self.return_queue.get_nowait()
        self.return_queue.put(curr_selected)

    def get_selected_options(self):
        to_ret = dict()
        to_ret['start'] = self.start_next_trial
        to_ret['quit'] = self.quit
        to_ret['method'] = self.method_selector.get_value()[0]
        to_ret['ui_device'] = self.device_selector.get_value()
        to_ret['record'] = self.record_next_trial
        to_ret['transition_function'] = partial(
            self.transition_function_selector.get_value(), gamma=self.method_selector.get_value()[1])
        to_ret['prediction_option'] = self.prediction_selector.get_value()
        return to_ret


def toggle_trial_button_callback(button, curr_val):
    to_ret = not curr_val
    if to_ret:
        configure_button_selected(button)
    else:
        configure_button_not_selected(button)
    return to_ret


def configure_button_selected(button):
    button.configure(bg="#cc0000", activebackground="#ff0000")


def configure_button_not_selected(button):
    global default_bg_color
    button.configure(bg=default_bg_color, activebackground="white")


def create_gui(get_gui_state_event, trial_starting_event, data_queue):
    gui = GuiHandler(get_gui_state_event, trial_starting_event, data_queue)
    import signal
    import sys
    signal.signal(signal.SIGTERM, lambda signum, stack_frame: sys.exit())

    gui.mainloop()


def start_gui_process():
    get_gui_state_event = multiprocessing.Event()
    trial_starting_event = multiprocessing.Event()
    data_queue = multiprocessing.Queue()
    p = multiprocessing.Process(target=create_gui, args=(
        get_gui_state_event, trial_starting_event, data_queue,))
    p.daemon = True
    p.start()

    return get_gui_state_event, trial_starting_event, data_queue, p


def empty_queue(queue):
    while not queue.empty():
        try:
            queue.get_nowait()
        except Empty:
            break
