import numpy as np
from model.transformations import get_rotation_transform, get_translation_transform
import random

name_counter = 0

class Element(object):
    def __init__(self, name=None):
        global name_counter

        self.parent_element = None
        self.child_elements = {}
        self.vectors = {}
        self.mapped_vectors = {}
        if name is not None:
            self.name = name
        else:
            self.name = str(name_counter)
            name_counter += 1

    def add(self, element):
        element.parent_element = self
        if element.name in self.child_elements:
            raise ValueError('element with name' + element.name + ' already exist')
        self.child_elements[element.name] = element

    def _get(self, n):
        if len(n) == 0:
            return self
        if n[0] not in self.child_elements:
            return None
        return self.child_elements[n[0]]._get(n[1:])

    def get(self, name_path):
        e = self._get(name_path.split('.'))
        if e is None:
            raise ValueError('No element: ' + name_path)
        return e

    def traverse_model(self, base_name=''):
        for e in self.child_elements.values():
            name = (base_name + '.' + e.name) if base_name != '' else (base_name + e.name)
            for x, n in e.traverse_model(name):
                yield x, n
        yield self, base_name

    def _get_parameters(self):
        return set()

    def remove_shapes(self):
        self.child_elements = {n: e for n, e in self.child_elements.items()
                               if not isinstance(self.child_elements[n], Shape)}
        for e in self.child_elements.values():
            e.remove_shapes()

    def prune_not_containing(self, keep_elements, name=''):
        '''
        :param keep_elements:
        :param name:
        :return: True = prune this branch
        '''
        if name in keep_elements:
            return False

        self.child_elements = {n: e for n, e in self.child_elements.items()
                               if not e.prune_not_containing(keep_elements,
                                                             name + ('.' if name is not '' else '') + n)}
        return len(self.child_elements) == 0

    def reset_mapped_coords(self):
        self.mapped_vectors = {}
        for n, v in self.vectors.items():
            self.mapped_vectors[n] = np.copy(v)
        for e in self.child_elements.values():
            e.reset_mapped_coords()

    def act(self):
        for e in self.child_elements.values():
            e.act()
        self._act()

    def _act(self):
        pass

    def transform(self, t):

        for n, v in self.vectors.items():
            t(v)
        for e in self.child_elements.values():
            e.transform(t)

    def transform_mapped(self, t):
        for n, v in self.mapped_vectors.items():
            t(v)
        for e in self.child_elements.values():
            e.transform_mapped(t)

    def draw(self, canvas):
        self._draw(canvas)
        for e in self.child_elements.values():
            e.draw(canvas)

    def _draw(self, canvas):
        pass


class Model(Element):
    def __init__(self, name=None):
        Element.__init__(self, name)

    def calc_coords(self):
        self.reset_mapped_coords()
        self.act()


class Bone(Element):
    def __init__(self, name=None):
        Element.__init__(self, name)
        self.connection_points = {}

    def add_connection_point(self, name, coords):
        self.vectors['connection_point:'+name] = np.array(coords, ndmin=2)
        self.connection_points[name] = []

    def get_connection_point_mapped_vector(self, name):
        return self.mapped_vectors['connection_point:'+name]

    def add(self, element, connection_point=None):
        if connection_point is not None:
            self.connection_points[connection_point].append(element.name)
        Element.add(self, element)

    def _act(self):
        for connection_point, elements in self.connection_points.items():
            t = get_translation_transform(self.vectors['connection_point:'+connection_point].reshape((3,)))
            for element_name in elements:
                # this might be broken if the model is pruned
                if element_name in self.child_elements:
                    self.child_elements[element_name].transform_mapped(t)


class RotationJoint(Element):
    def __init__(self, origin, axis, angle, name=None):
        Element.__init__(self, name)
        self.vectors['origin'] = np.array(origin, ndmin=2)
        self.vectors['axis'] = np.array(axis, ndmin=2)
        self.angle = angle

    def _act(self):
        t = get_rotation_transform(self.mapped_vectors['axis'].reshape((3,)),
                                                      self.angle,
                                                      self.mapped_vectors['origin'].reshape((3,)))
        for e in self.child_elements.values():
            e.transform_mapped(t)


    def _draw(self, canvas):
        canvas.create_oval(self.mapped_vectors['origin'][0, 0] - 2,
                           self.mapped_vectors['origin'][0, 1] - 2,
                           self.mapped_vectors['origin'][0, 0] + 2,
                           self.mapped_vectors['origin'][0, 1] + 2)


class Target(Element):
    def __init__(self, position, name=None):
        Element.__init__(self, name)
        self.vectors['position'] = np.array(position, ndmin=2)

    def _draw(self, canvas):
        canvas.create_oval(self.mapped_vectors['position'][0, 0] - 4,
                           self.mapped_vectors['position'][0, 1] - 4,
                           self.mapped_vectors['position'][0, 0] + 4,
                           self.mapped_vectors['position'][0, 1] + 4)


class Shape(Element):
    def __init__(self, vertices, lines, name=None):
        Element.__init__(self, name)
        self.vectors['vertices'] = vertices
        self.lines = lines

    def add(self, element):
        if not isinstance(element, Shape):
            raise ValueError('A Shape should not have non-shape child elements')
        Element.add(self, element)

    def _draw(self, canvas):
        for v1, v2 in self.lines:
            if self.mapped_vectors['vertices'][v1, 2] > 0 and self.mapped_vectors['vertices'][v2, 2] > 0:
                canvas.create_line(self.mapped_vectors['vertices'][v1, 0],
                                   self.mapped_vectors['vertices'][v1, 1],
                                   self.mapped_vectors['vertices'][v2, 0],
                                   self.mapped_vectors['vertices'][v2, 1])
