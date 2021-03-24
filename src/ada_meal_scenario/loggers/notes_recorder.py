import functools

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk # python3

NOTES_CONFIG_NAME = "notes"
NUM_GOALS = 2

class NotesLoggerFrame(tk.LabelFrame, object):
    def __init__(self, parent, initial_config={}):
        super(NotesLoggerFrame, self).__init__(parent, text='Notes')
        initial_config = initial_config.get(NOTES_CONFIG_NAME, {})

        self.goal_choice_label = tk.Label(self, text='Declared goal:')
        self.goal_choice_label.grid(row=0, column=0, sticky='nw')

        # TODO: actually match this to the tag detection frame somehow?
        # also: match index to actual goal
        self.goal_choices_frame = tk.Frame(self)
        self.goal_choices_frame.grid(row=1, column=0, sticky='nsew')
        self.goal_choice_buttons = []
        self.goal_choice = -1
        for i in range(NUM_GOALS):
            goal_choice = tk.Button(self.goal_choices_frame, text=str(i), command=functools.partial(self._choose_goal, i))
            goal_choice.grid(row=0, column=i, sticky='nsew')
            self.goal_choice_buttons.append(goal_choice)

    def _choose_goal(self, i):
        if self.goal_choice >= 0:
            self.goal_choice_buttons[self.goal_choice].configure(relief=tk.RAISED)
        if i != self.goal_choice:
            self.goal_choice_buttons[i].configure(relief=tk.SUNKEN)            
            self.goal_choice = i
        else:
            self.goal_choice = -1

    def get_config(self):
        return { NOTES_CONFIG_NAME: {
                'goal_choice': self.goal_choice
            }
        }
    
    def set_state(self, state):
        for gc in self.goal_choice_buttons:
            gc.configure(state=state)

    

