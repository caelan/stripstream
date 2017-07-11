from Tkinter import Tk, Toplevel, Canvas
import numpy as np
from numpy.core.multiarray import array
import random


class PRMViewer(object):

    def __init__(self, width=500, height=500, title='PRM', background='tan'):
        tk = Tk()
        tk.withdraw()
        top = Toplevel(tk)
        top.wm_title(title)
        top.protocol('WM_DELETE_WINDOW', top.destroy)
        self.width = width
        self.height = height
        self.canvas = Canvas(top, width=self.width,
                             height=self.height, background=background)
        self.canvas.pack()

    def pixel_from_point(self, (x, y)):
        # return (int(x*self.width), int(self.height - y*self.height))
        return (x * self.width, self.height - y * self.height)

    def draw_point(self, point, radius=5):
        (x, y) = self.pixel_from_point(point)
        self.canvas.create_oval(x - radius, y - radius,
                                x + radius, y + radius, fill='black')

    def draw_line(self, (point1, point2)):
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_line(x1, y1, x2, y2, fill='black', width=2)

    def draw_rectangle(self, (point1, point2), width=1, color='brown'):
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, width=2)

    def clear(self):
        self.canvas.delete('all')

#################################################################


def get_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))


def contains(point, box):
    (bp1, bp2) = box
    return np.all(array(point) >= array(bp1)) and np.all(array(bp2) >= array(point))


def sample_line((point1, point2), step_size=.05):
    for l in np.arange(0., 1., step_size):
        yield tuple(l * np.array(point1) + (1 - l) * np.array(point2))
    yield point2


def line_collides(line, box):  # TODO - could also compute this exactly
    return any(contains(p, box) for p in sample_line(line))


def is_collision_free(line, boxes):
    return not any(line_collides(line, box) for box in boxes)


def create_box((x, y), width, height):
    return ((x - width / 2., y - height / 2.), (x + width / 2., y + height / 2.))


def sample_box((point1, point2)):
    return tuple(np.random.random(2) * (np.array(point2) - np.array(point1)) + np.array(point1))


def is_region(goal):
    return isinstance(goal[0], tuple) and isinstance(goal[1], tuple)


def sample(digits=3):
    return (round(random.random(), digits),
            round(random.random(), digits))


def inf_sequence():
    return iter(int, 1)


def inf_sampler(sampler):
    for _ in inf_sequence():
        yield sampler()


def draw_solution(plan, goal, obstacles):
    viewer = PRMViewer()
    for box in obstacles:
        viewer.draw_rectangle(box, color='brown')
    if is_region(goal):
        viewer.draw_rectangle(goal, color='green')
    if plan:
        for _, line in plan:
            viewer.draw_line(line)
            # for p in [p1, p2]:
            for p in sample_line(line):
                viewer.draw_point(p, radius=2)
