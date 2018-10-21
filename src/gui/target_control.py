from tkinter import *
import numpy as np


class TargetControl(object):
    def __init__(self, container, model, target_name, canvas3d, inverse_kinematics):
        self.container = container
        self.model = model
        self.target_name = target_name
        self.canvas3d = canvas3d
        self.inverse_kinematics = inverse_kinematics

        label = Label(container, text=target_name)
        label.grid(row=0, column=0, columnspan=3)

        # add controls
        self.b_left = Button(container, text="<", command=lambda: self.move_target([0, 0, 10]))
        self.b_right = Button(container, text=">", command=lambda: self.move_target([0, 0, -10]))
        self.b_up = Button(container, text="^", command=lambda: self.move_target([0, -10, 0]))
        self.b_down = Button(container, text="v", command=lambda: self.move_target([0, 10, 0]))
        self.b_in = Button(container, text="-", command=lambda: self.move_target([10, 0, 0]))
        self.b_out = Button(container, text="+", command=lambda: self.move_target([-10, 0, 0]))

        self.b_left.grid(row=1, column=0)
        self.b_right.grid(row=2, column=0)
        self.b_up.grid(row=1, column=1)
        self.b_down.grid(row=2, column=1)
        self.b_in.grid(row=1, column=2)
        self.b_out.grid(row=2, column=2)

    def move_target(self, move_v):
        target = self.model.get(self.target_name)
        v = target.vectors['position']
        v[:] = v + np.array(move_v)
        self.inverse_kinematics.search(max_iterations=20, apply=True)

        self.canvas3d.redraw()
