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

class GuiHandler(object):
    def __init__(self, get_gui_state_event, trial_starting_event, return_queue):
        self.master = Tkinter.Tk()
        #self.method = None
        #self.ui_device = None
        self.all_buttons = dict()
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

        self.method_frame = Tkinter.Frame(self.master)
        self.method_frame.grid(row=0, column=0, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)
        self.init_method_buttons(self.method_frame)

        empty_frame = Tkinter.Frame(self.master)
        empty_frame.grid(row=0, column=1, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)
        empty_label = Tkinter.Label(empty_frame, text="     ")
        empty_label.grid(sticky=Tkinter.W+Tkinter.E)

        self.UI_frame = Tkinter.Frame(self.master)
        self.UI_frame.grid(row=0, column=2, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)
        self.init_ui_device_buttons(self.UI_frame)

        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(row=1, column=2, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)

        self.record_button = Tkinter.Button(self.start_frame, text="Record Next Trial", command=self.record_button_callback)
        self.record_button.grid(sticky=Tkinter.W+Tkinter.E)
        self.start_button = Tkinter.Button(self.start_frame, text="Start Next Trial", command=self.start_button_callback)
        self.start_button.grid(sticky=Tkinter.W+Tkinter.E)


        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(row=1, column=0, sticky=Tkinter.W+Tkinter.E+Tkinter.S)
        self.quit_button = Tkinter.Button(self.start_frame, text="Quit", command=self.quit_button_callback)
        self.quit_button.grid(sticky=Tkinter.W+Tkinter.E)

        #self.all_buttons[arg] = b

        self.method='autonomous'
        self.ui_device='kinova'
        self.color_buttons()

    def mainloop(self):
        #self.master.mainloop()
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

    def init_transition_buttons(self, frame):
        label_font = self.default_font.copy()
        label_font.configure(weight='bold')
        self.method_label = Tkinter.Label(frame, text="Transition Function: \n", font=label_font)
        self.method_label.grid(sticky=Tkinter.W+Tkinter.E)

        self.button_2gamma = self.init_button_with_callback(self.select_transition_function,
                                                            lambda x,y,gamma: (2-2*gamma)*x+2*gamma*y,
                                                            '(2-2*gamma)*a + 2*gamma*u',
                                                            frame)
        self.button_1gamma = self.init_button_with_callback(self.select_transition_function,
                                                            lambda x,y,gamma: (1-gamma)*x+gamma*y,
                                                            '(1-gamma)*a + gamma*u',
                                                            frame)
    def init_method_buttons(self, frame):
        label_font = self.default_font.copy()
        label_font.configure(weight='bold')
        self.method_label = Tkinter.Label(frame, text="Method: \n", font=label_font)
        self.method_label.grid(sticky=Tkinter.W+Tkinter.E)

        self.button_direct_teleop =  self.init_button_with_callback(self.select_assistance_method, 
                                                                    ['direct', 0.0],
                                                                    'Direct Teleop', 
                                                                    frame)        
        self.button_shared_auton_1 = self.init_button_with_callback(self.select_assistance_method, 
                                                                    ['shared_auton_1', 0.33], 
                                                                    'Shared Auton Lvl 1', 
                                                                    frame)
        self.button_shared_auton_2 = self.init_button_with_callback(self.select_assistance_method, 
                                                                    ['shared_auton_2', 0.66],
                                                                    'Shared Auton Lvl 2', 
                                                                    frame)
        self.button_full_auton = self.init_button_with_callback(self.select_assistance_method, 
                                                                ['autonomous', 1.0],
                                                                'Fully Autonomous', 
                                                                frame)

    def init_ui_device_buttons(self, frame):
        label_font = self.default_font.copy()
        label_font.configure(weight='bold')
        self.method_label = Tkinter.Label(frame, text="UI Device \n", font=label_font)
        self.method_label.grid(sticky=Tkinter.W+Tkinter.E)
        
        self.button_mouse = self.init_button_with_callback(self.select_ui_device, 'mouse', 'Mouse', frame)
        self.button_kinova = self.init_button_with_callback(self.select_ui_device, 'kinova', 'Kinova USB', frame)

    def init_button_with_callback(self, func, args, label, frame):
        callback = partial(func, args)
        b = Tkinter.Button(frame, text=label, command=callback)
        b.grid(sticky=Tkinter.W+Tkinter.E)
        self.all_buttons[arg] = b

        #b.configure(state = "normal", relief="raised")

        return b

    def select_transition_function(self, transition_function):
        self.transition_function = transition_function
        self.color_buttons()

    def select_assistance_method(self, args):
        self.method = args[0]
        self.gamma = args[1]
        #print 'method: ' + str(method)
        self.color_buttons()

    def select_ui_device(self, ui_device):
        self.ui_device = ui_device
        #print 'ui: ' + str(ui_device)
        self.color_buttons()

    def start_button_callback(self):
        self.start_next_trial = toggle_trial_button_callback(self.start_button, self.start_next_trial)
        #self.add_return_to_queue()

    def record_button_callback(self):
        self.record_next_trial = toggle_trial_button_callback(self.record_button, self.record_next_trial)

    def quit_button_callback(self):
        self.quit = toggle_trial_button_callback(self.quit_button, self.quit)
        #self.add_return_to_queue()

    def add_return_to_queue(self):
        curr_selected = self.get_selected_options()
        #while not self.return_queue.empty():
        #    self.return_queue.get_nowait()
        self.return_queue.put(curr_selected)


    def get_selected_options(self):
        to_ret = dict()
        to_ret['start'] = self.start_next_trial
        to_ret['quit'] = self.quit
        to_ret['method'] = self.method
        to_ret['ui_device'] = self.ui_device
        to_ret['record'] = self.record_next_trial
        to_ret['transition_function'] = partial(self.transition_function, gamma=self.gamma)
        return to_ret

    def color_buttons(self):
        #reset all
        for key,b in self.all_buttons.iteritems():
            configure_button_not_selected(b)
        #set the selected items
        configure_button_selected(self.all_buttons[self.ui_device])
        configure_button_selected(self.all_buttons[self.method])


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
    p = multiprocessing.Process(target=create_gui, args=(get_gui_state_event, trial_starting_event, data_queue,))
    p.daemon = True
    p.start()

    return get_gui_state_event, trial_starting_event, data_queue, p
    

def empty_queue(queue):
    while not queue.empty():
        try:
            queue.get_nowait()
        except Empty:
            break






