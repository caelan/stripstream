from PyR2 import hu
from PyR2.shapes import BoxAligned, Shape

from itertools import count
import math
import numpy as np


def get_window_name(window_3d):
    return window_3d.window.title


def is_window_active(window_3d):
    return window_3d.window is not None


def get_name(shape):
    return shape.name()


def get_color(shape):
    return shape.properties['color']


def get_box_body(length, width, height, name='box', color='black'):
    return Shape([
        BoxAligned(np.array([(-length / 2., -width / 2., -height / 2),
                             (length / 2., width / 2., height / 2)]), None)
    ], None, name=name, color=color)


GRASP_FRAME = hu.Pose(0, -0.025, 0, 0)
CENTER_FRAME = hu.Pose(0, 0, 0.025, 0)


def get_grasp_frame_origin(grasp_frame):

    objFrame = hu.Pose(0, 0, 0, 0)
    faceFrame = grasp_frame.compose(GRASP_FRAME)
    centerFrame = faceFrame.compose(CENTER_FRAME)
    graspFrame = objFrame.compose(centerFrame)
    return graspFrame


Z_OFFSET = 0.02


gMat0 = hu.Transform(np.array([(0., 1., 0., 0.),
                               (0., 0., 1., -0.025),
                               (1., 0., 0., Z_OFFSET),
                               (0., 0., 0., 1.)]))
gMat1 = hu.Transform(np.array([(0., -1., 0., 0.),
                               (0., 0., -1., 0.025),
                               (1., 0., 0., Z_OFFSET),
                               (0., 0., 0., 1.)]))
gMat4 = hu.Pose(0, 0, 0, math.pi / 2).compose(gMat0)
gMat5 = hu.Pose(0, 0, 0, -math.pi / 2).compose(gMat0)

SIDE_GRASPS = map(get_grasp_frame_origin, [gMat0, gMat1, gMat4, gMat5])


gMat2 = hu.Transform(np.array([(-1., 0., 0., 0.),
                               (0., 0., -1., 0.025),
                               (0., -1., 0., 0.),
                               (0., 0., 0., 1.)]))
gMat3 = hu.Transform(np.array([(1., 0., 0., 0.),
                               (0., 0., 1., -0.025),
                               (0., -1., 0., 0.),
                               (0., 0., 0., 1.)]))

TOP_GRASPS = map(get_grasp_frame_origin, [gMat2, gMat3])


def is_left_active(robot):
    return 'pr2LeftArm' in robot.moveChainNames


def is_right_active(robot):
    return 'pr2RightArm' in robot.moveChainNames


def open_grippers(conf):
    for arm in conf.robot.gripperChainNames:
        conf.set(conf.robot.gripperChainNames[arm], [conf.robot.gripMax])


def close_grippers(conf):
    for arm in conf.robot.gripperChainNames:
        conf.set(conf.robot.gripperChainNames[arm], [0.0])


class Traj(object):
    _ids = count(0)

    def __init__(self, path, grasp=None):
        self.id = next(self._ids)
        self.path = path
        self.grasp = grasp

    def __repr__(self):
        return '%s(%d)' % (self.__class__.__name__, self.id)


def interpolate(q1, q2, step_size, active_chains=None):
    assert q1.robot == q2.robot
    moveChains = active_chains or q1.keys()
    q = q1
    while not all(q[c] == q2[c] for c in moveChains):
        yield q
        q = q1.robot.stepAlongLine(
            q2, q, step_size, forward=True, moveChains=moveChains)
    yield q2


def forward_kinematics(conf, arm):
    return conf.cartConf()[conf.robot.armChainNames[arm]]


def inverse_kinematics(default_conf, manip_trans, arm):
    return default_conf.robot.inverseKinWristGen(manip_trans, arm, default_conf,
                                                 basePose=default_conf.basePose())


def get_wrist_frame(grasp, robot, hand):
    gripperFrame = robot.gripperFaceFrame[hand]
    wristFrame = grasp.compose(gripperFrame.inverse())
    return wristFrame


def manip_from_pose_grasp(pose, grasp, robot, hand):
    return pose.compose(get_wrist_frame(grasp, robot, hand))


def pose_from_manip_grasp(manip, grasp, robot, hand):
    return manip.compose(get_wrist_frame(grasp, robot, hand).inverse())

BACKOFF = 0.15
PERP_BACKOFF = 0.05


def approach_from_manip(manip_trans):
    if abs(manip_trans.matrix[2, 0]) < 0.1:
        return manip_trans.compose(hu.Pose(-BACKOFF, 0., PERP_BACKOFF, 0.))
    return manip_trans.compose(hu.Pose(-BACKOFF, 0., 0., 0.))


def get_sample_fn(conf, active_chains):
    return lambda: conf.robot.randomConf(active_chains, conf)


def get_extend_fn(active_chains, step_size):

    return lambda q1, q2: interpolate(q1, q2, step_size, active_chains)


def get_distance_fn(active_chains):
    def distance_fn(q1, q2):
        assert q1.robot == q2.robot
        return q1.robot.distConf(q1, q2, active_chains)

    return distance_fn


def get_collision_fn(placed=[], attached=None):

    def collision_fn(q):
        placed_robot = q.placement(attached)

        return any(placed_robot.collides(body) for body in placed)
    return collision_fn


ROBOT_COLOR = 'gold'


def draw_window_3d(window_3d, robot_conf, placements, attached=None):
    if not is_window_active(window_3d):
        return
    window_3d.clear()
    placed_robot = robot_conf.placement(attached=attached)
    window_3d.draw(placed_robot, color=ROBOT_COLOR)
    for placed in placements:
        window_3d.draw(placed, color=get_color(placed))
