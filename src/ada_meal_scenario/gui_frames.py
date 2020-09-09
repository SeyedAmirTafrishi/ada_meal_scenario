import Tkinter
import tkFont


class OptionSelector(Tkinter.Frame, object):
    _BUTTON_CONFIG = {
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
                                         **OptionSelector._BUTTON_CONFIG)
            button.grid(row=1+idx, column=0, sticky=Tkinter.E+Tkinter.W)
            self.buttons.append(button)

        if default_selection is not None:
            self.buttons[default_selection].select()

    def get_value(self):
        return self.option_values[self.variable.get()]

    def set_state(self, state):
        for button in self.buttons:
            button.configure(state=state)
