from tkinter import *
from model.skeleton_model import *
from model.shapes import *
from gui.canvas_3d import Canvas3D
from gui.target_control import TargetControl
from model.inverse_kinematics import InverseKinematics

def build_model():
    model = Model()
    body = get_robot_body('body')
    right_arm = get_robot_arm('right_arm')
    left_arm = get_robot_arm('left_arm')

    model.add(body)
    body.add(right_arm, connection_point='right_arm')
    body.add(left_arm, connection_point='left_arm')

    target_left = Target([0, 0, 100], name='left_hand_target')
    target_right = Target([0, 0, -100], name='right_hand_target')
    model.add(target_left)
    model.add(target_right)
    return model

model = build_model()

left_arm_target_name = 'body.left_arm.shoulder_joint1.shoulder_joint2.shoulder_joint3.upper_arm.elbow_joint.lower_arm'
left_arm_target_connection_point = 'joint'
left_arm_joint_names = {'body.left_arm.shoulder_joint1',
                        'body.left_arm.shoulder_joint1.shoulder_joint2',
                        'body.left_arm.shoulder_joint1.shoulder_joint2.shoulder_joint3',
                        'body.left_arm.shoulder_joint1.shoulder_joint2.shoulder_joint3.upper_arm.elbow_joint'}
ik_left = InverseKinematics(model, left_arm_target_name,
                            left_arm_target_connection_point,
                            'left_hand_target',
                            joint_names=left_arm_joint_names)
ik_left.search(apply=True)


right_arm_target_name = 'body.right_arm.shoulder_joint1.shoulder_joint2.shoulder_joint3.upper_arm.elbow_joint.lower_arm'
right_arm_target_connection_point = 'joint'
right_arm_joint_names = {'body.right_arm.shoulder_joint1',
                         'body.right_arm.shoulder_joint1.shoulder_joint2',
                         'body.right_arm.shoulder_joint1.shoulder_joint2.shoulder_joint3',
                         'body.right_arm.shoulder_joint1.shoulder_joint2.shoulder_joint3.upper_arm.elbow_joint'}
ik_right = InverseKinematics(model, right_arm_target_name,
                             right_arm_target_connection_point,
                             'right_hand_target',
                             joint_names=right_arm_joint_names)
ik_right.search(apply=True)

master = Tk()
master.title("ROBOT Control")

canvas_width = 500
canvas_height = 300
canvas = Canvas(master,
           width=canvas_width,
           height=canvas_height)
canvas.grid(row=0, column=0, rowspan=5, columnspan=1, sticky=N+S+E+W)
master.columnconfigure(0, weight=1)
master.rowconfigure(4, weight=1)

canvas3d = Canvas3D(model, canvas)
canvas3d.redraw()

target_control_frame_left = Frame()
target_control_frame_left.grid(row=0, column=1, rowspan=1, columnspan=1)
target_control_left = TargetControl(target_control_frame_left,
                                    model,
                                    'left_hand_target',
                                    canvas3d,
                                    ik_left)

target_control_frame_right = Frame()
target_control_frame_right.grid(row=1, column=1, rowspan=1, columnspan=1)
target_control_right = TargetControl(target_control_frame_right,
                                     model,
                                     'right_hand_target',
                                     canvas3d,
                                     ik_right)

mainloop()