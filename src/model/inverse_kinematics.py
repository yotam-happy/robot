'''
Finds best path to move the joints of a model in order for a target point to reach a desired
position in space.

The search is really simple and primitive... it is done using simple beam search of the model parameters.
It is probably only good for simple models with few parameters.
TODO: this should return the entire movement path, not only the target parameters
TODO: should be able to consider arbitrary search space constraints
'''

import copy
import random
import numpy as np
from model.skeleton_model import RotationJoint


class RotationJointAngleParam(object):
    def __init__(self, joint_name, base_angle, angle=None):
        self.joint_name = joint_name
        self.base_angle = base_angle
        self.angle = angle if angle is not None else base_angle

    def copy(self):
        return RotationJointAngleParam(self.joint_name, self.base_angle, self.angle)

    def apply_random_step(self, step):
        r = random.randrange(3)
        if r == 1:
            self.angle += step
        elif r == 2:
            self.angle -= step

    def apply_to_model(self, model):
        joint = model.get(self.joint_name)
        joint.angle = self.angle

    def reset_model(self, model):
        joint = model.get(self.joint_name)
        joint.angle = self.base_angle

    def cost(self):
        return abs(self.angle - self.base_angle)


def collect_parameters(model, joint_names=None):
    params = set()
    if joint_names is None:
        # collect all model parameters... This can add redundant parameters and make
        # them move randomly
        for element, name in model.traverse_model():
            if isinstance(element, RotationJoint):
                params.add(RotationJointAngleParam(name, element.angle))
    else:
        for name in joint_names:
            element = model.get(name)
            if isinstance(element, RotationJoint):
                params.add(RotationJointAngleParam(name, element.angle))
    return params


class InverseKinematics(object):
    def __init__(self, model, source_element, source_connection_point, target_name, joint_names=None):
        self.source_model = model
        self.source_element = source_element
        self.source_connection_point = source_connection_point
        self.target_name = target_name
        self.joint_names = joint_names

    def search(self,
               beam_size=10,
               n_next_steps=10,
               max_iterations=50,
               step=0.1,
               early_stop=2.0,
               apply=False):
        '''
        Really simple beam search...
        No bells and whistles. Not even parallel execution
        '''
        model_copy = copy.deepcopy(self.source_model)
        model_copy.remove_shapes()
        model_copy.prune_not_containing(self.joint_names | {self.target_name})
        target_position = model_copy.get(self.target_name).vectors['position']
        start = collect_parameters(model_copy, self.joint_names)
        beam = [{p.copy() for p in start} for i in range(beam_size)]
        gamma = 1.0
        step_mult = 1.0

        iterations = 0
        done = False
        while not done:
            # get next_step
            next_steps = []
            for i in range(n_next_steps):
                next_step = {p.copy() for p in beam[random.randrange(len(beam))]}
                for param in next_step:
                    param.apply_random_step(step * step_mult)
                next_steps.append(next_step)

            # narrow down beam
            beam.extend(next_steps)

            g_scores = [sum([p.cost() for p in params]) for params in beam]
            h_scores = [self.h(params, target_position, model_copy) for params in beam]
            sorted_beam = sorted(zip(g_scores, h_scores, beam), key=lambda x: gamma * x[0] + x[1])
            beam = [x for _, _, x in sorted_beam[:beam_size]]
            iterations += 1
            if iterations == max_iterations:
                done = True
            if early_stop is not None and sorted_beam[0][1] <= early_stop:
                done = True
            if iterations >= max_iterations / 2:
                gamma = 0.1
                step_mult = 0.2
        print('IK best cost:', sorted_beam[0][0])

        if apply:
            for p in beam[0]:
                p.apply_to_model(self.source_model)

        return beam[0]

    def h(self, params, position, model_copy):
        for p in params:
            p.apply_to_model(model_copy)
        model_copy.calc_coords()
        source = model_copy.get(self.source_element)
        source_vector = source.get_connection_point_mapped_vector(self.source_connection_point)
        return np.linalg.norm(position.flatten() - source_vector.flatten(), ord=2)
