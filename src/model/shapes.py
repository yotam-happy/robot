import numpy as np
from model.skeleton_model import Shape, Bone, RotationJoint
from model.transformations import get_translation_transform

def _flat_polygon(vertices):
    lines = []
    for i in range(vertices.shape[0]):
        lines.append((i, (i+1) % vertices.shape[0]))
    return np.array(vertices, dtype=float), lines


def flat_polygon(vertices):
    vertices, lines = flat_polygon(vertices)
    return Shape(vertices, lines)


def extruded_polygon(vertices, inflate_vector):
    vertices1, lines1 = _flat_polygon(vertices)
    vertices = np.zeros((vertices1.shape[0] * 2, vertices1.shape[1]))
    vertices[:vertices1.shape[0], :] = vertices1 - inflate_vector / 2
    vertices[vertices1.shape[0]:, :] = vertices1 + inflate_vector / 2
    lines = list(lines1)
    lines.extend([(st + vertices1.shape[0], en + vertices1.shape[0]) for st, en in lines1])
    for i in range(vertices1.shape[0]):
        lines.append((i, i + vertices1.shape[0]))
    return Shape(vertices, lines)


def get_robot_arm_section(name=None, length=100):
    shape = extruded_polygon(np.array([[-10, 0, 0],
                                       [0, length, 0],
                                       [10, 0, 0]]),
                             np.array([0, 0, 10]))
    bone = Bone(name)
    bone.add(shape)
    bone.add_connection_point('joint', np.array([0, length, 0]))
    return bone

def get_robot_arm(name=None):
    right_arm1 = get_robot_arm_section('upper_arm')
    right_arm2 = get_robot_arm_section('lower_arm')
    shoulder1 = RotationJoint(np.array([0, 0, 0]),
                              np.array([0, 0, 1]),
                              0, 'shoulder_joint1')
    shoulder2 = RotationJoint(np.array([0, 0, 0]),
                              np.array([1, 0, 0]),
                              0, 'shoulder_joint2')
    shoulder3 = RotationJoint(np.array([0, 0, 0]),
                              np.array([0, 1, 0]),
                              0, 'shoulder_joint3')
    elbow = RotationJoint(np.array([0, 0, 0]),
                          np.array([0, 0, 1]),
                          0, 'elbow_joint')

    shoulder1.add(shoulder2)
    shoulder2.add(shoulder3)
    shoulder3.add(right_arm1)
    elbow.add(right_arm2)
    right_arm1.add(elbow, connection_point='joint')

    bone = Bone(name)
    bone.add(shoulder1)

    return bone


def get_robot_body(name=None):
    shape = extruded_polygon(np.array([[-100, 0, 0],
                                      [-100, -40, 0],
                                      [50, -90, 0],
                                      [150, -140, 0],
                                      [200, -140, 0],
                                      [130, -40, 0],
                                      [150, 0, 0]]), np.array([0, 0, 100]))
    bone = Bone(name)
    bone.add(shape)
    bone.add_connection_point('right_arm', np.array([150, -120, -55]))
    bone.add_connection_point('left_arm', np.array([150, -120, 55]))
    bone.add_connection_point('head', np.array([175, -145, 0]))
    bone.transform(get_translation_transform(np.array([0, 50, 0])))
    return bone

