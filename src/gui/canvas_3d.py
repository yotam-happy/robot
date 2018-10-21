import math
import numpy as np
from model.transformations import *

class Canvas3D(object):
    def __init__(self, model, canvas):
        self.model = model
        self.canvas = canvas
        self.view_angle_x_axis = 0.2
        self.view_angle_y_axis = - math.pi / 4
        self.frame_size = 300

        self.drag_origin_xy = None
        self.view_angle_origin_xy = None

        canvas.bind("<Configure>", lambda e: self.redraw())
        canvas.bind("<ButtonPress-1>", lambda e: self.drag_set_origin_xy(e))
        canvas.bind("<B1-Motion>", lambda e: self.drag_rotate_canvas(e))

    def drag_set_origin_xy(self, e):
        self.drag_origin_xy = (e.x, e.y)
        self.view_angle_origin_xy = (self.view_angle_x_axis, self.view_angle_y_axis)

    def drag_rotate_canvas(self, e):
        drag_x, drag_y = self.drag_origin_xy
        drag_x = e.x - drag_x
        drag_y = e.y - drag_y
        self.view_angle_x_axis, self.view_angle_y_axis = self.view_angle_origin_xy
        self.view_angle_x_axis += drag_y / 100
        self.view_angle_y_axis -= drag_x / 100
        self.redraw()


    def redraw(self):
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        scale_factor = np.min([canvas_height, canvas_width]) / self.frame_size

        self.canvas.delete("all")
        self.model.calc_coords()

        # camera transforms
        self.model.transform_mapped(get_rotation_transform(np.array([0, 1, 0]),
                                                           self.view_angle_y_axis))
        self.model.transform_mapped(get_rotation_transform(np.array([1, 0, 0]),
                                                           self.view_angle_x_axis))
        self.model.transform_mapped(get_translation_transform(np.array([0, 0, 300])))
        self.model.transform_mapped(get_fake_perspective_transform(300))

        # fit to display coords
        self.model.transform_mapped(get_scale_transform(np.array([scale_factor, scale_factor, scale_factor])))
        self.model.transform_mapped(get_translation_transform(np.array([canvas_width / 2, canvas_height / 2, 0])))

        self.model.draw(self.canvas)
        # view_angle += math.pi / 300
        # joint1.angle += math.pi / 100
        #self.model.get('body.left_arm.shoulder_joint').angle += math.pi / 50
