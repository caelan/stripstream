from itertools import product
from random import sample
import math

import numpy as np

from PyR2 import hu
from robotics.pyr2.utils import get_box_body
from PyR2.pr2.pr2Robot import makeRobot

BODY_PLACEMENT_Z_OFFSET = 1e-3
TOP_HOLDING_LEFT_ARM = [0.67717021, -0.34313199,
                        1.2, -1.46688405, 1.24223229, -1.95442826, 2.22254125]


class ManipulationProblem:

    def __init__(self, workspace, initial_conf, initial_poses=[],
                 movable_names=[], known_poses=[],
                 goal_conf=None, goal_poses={}):
        self.workspace = workspace
        self.initial_conf = initial_conf
        self.initial_poses = initial_poses
        self.movable_names = movable_names
        self.goal_conf = goal_conf
        self.goal_poses = goal_poses
        self.known_poses = known_poses


def dantam2():
    m, n = 3, 3

    n_obj = 2
    side_dim = .07
    height_dim = .1
    box_dims = (side_dim, side_dim, height_dim)
    separation = (side_dim, side_dim)

    workspace = np.array([[-2.0, -1.5, 0.0],
                          [1.0, 1.5, 2.0]])
    robot = makeRobot(workspace, useLeft=True, useRight=False)
    base_conf = (-.75, .2, -math.pi / 2)
    initial_conf = robot.makeConf(
        *base_conf, g=robot.gripMax, ignoreStandard=True)
    initial_conf = initial_conf.set('pr2LeftArm', TOP_HOLDING_LEFT_ARM)

    initial_poses = []
    goal_poses = {}

    length = m * (box_dims[0] + separation[0])
    width = n * (box_dims[1] + separation[1])
    height = .7
    table = get_box_body(length, width, height, name='table', color='brown')
    initial_poses.append((table, hu.Pose(0, 0, height / 2, 0)))

    poses = []
    z = height + height_dim / 2 + BODY_PLACEMENT_Z_OFFSET
    theta = 0
    for r in range(m):
        row = []
        x = -length / 2 + (r + .5) * (box_dims[0] + separation[0])
        for c in range(n):
            y = -width / 2 + (c + .5) * (box_dims[1] + separation[1])
            row.append(hu.Pose(x, y, z, theta))
        poses.append(row)

    coordinates = list(product(range(m), range(n)))
    assert n_obj <= len(coordinates)
    obj_coordinates = sample(coordinates, n_obj)
    movable_names = []
    for i, (r, c) in enumerate(obj_coordinates):
        row_color = np.zeros(4)
        row_color[2 - r] = 1.
        if i == 0:
            name = 'goal%d-%d' % (r, c)
            color = 'blue'

            goal_poses[name] = poses[m / 2][n / 2]
        else:
            name = 'block%d-%d' % (r, c)
            color = 'red'

        obj = get_box_body(*box_dims, name=name, color=color)
        initial_poses.append((obj, poses[r][c]))
        movable_names.append(name)

    known_poses = [pose for col in poses for pose in col]

    return ManipulationProblem(workspace, initial_conf, initial_poses=initial_poses,
                               movable_names=movable_names, known_poses=known_poses,
                               goal_poses=goal_poses, goal_conf=initial_conf)
