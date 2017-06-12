from collections import namedtuple
from random import random
from time import sleep
from stripstream.utils import random_sequence, irange


SUCTION_WIDTH = 1.5
SUCTION_HEIGHT = 1.
STEM_WIDTH = 1.
STEM_HEIGHT = 2.5


EPSILON = (SUCTION_WIDTH - 1.) / 2
print 'Epsilon:', EPSILON

COLLISION_TIME = 0
CONTAIN_TIME = 0
REGION_TIME = 0
IK_TIME = 0


Block = namedtuple('Block', ['name', 'w', 'h', 'color'])
Region = namedtuple('Region', ['name', 'x', 'w'])


def are_colliding(block1, x1, block2, x2):
    sleep(COLLISION_TIME)
    if x1 == x2:
        return True
    return not (x1 + block1.w / 2. < x2 - block2.w / 2. or
                x1 - block1.w / 2. > x2 + block2.w / 2.)


def in_region(region, block, x):
    sleep(CONTAIN_TIME)
    return region.x - region.w / 2. <= x - block.w / 2 and region.x + region.w / 2. >= x + block.w / 2


def sample_region(region):
    sleep(REGION_TIME)
    return region.x + region.w * (random() - .5)


def sample_region_pose(region, block):
    sleep(REGION_TIME)
    return region.x + (region.w - block.w) * (random() - .5)


def inverse_kinematics(pose):
    sleep(IK_TIME)
    return pose


def sample_inverse_kinematics(pose):
    sleep(IK_TIME)
    return pose + 2 * EPSILON * (random() - .5)


def is_inverse_kinematics(pose, config):
    sleep(IK_TIME)
    return abs(pose - config) <= EPSILON


TMPProblem = namedtuple('TMPProblem', ['env_region', 'initial_config', 'initial_holding', 'initial_poses',
                                       'goal_config', 'goal_holding', 'goal_poses', 'goal_regions'])


class TMPProblem(object):

    def __init__(self, env_region, initial_config, initial_holding, initial_poses,
                 goal_config, goal_holding, goal_poses, goal_regions):
        self.env_region = env_region
        self.initial_config = initial_config
        self.initial_holding = initial_holding
        self.initial_poses = initial_poses
        self.goal_config = goal_config
        self.goal_holding = goal_holding
        self.goal_poses = goal_poses
        self.goal_regions = goal_regions

    def get_blocks(self):
        blocks = self.initial_poses.keys()
        if self.initial_holding is not None:
            blocks.append(self.initial_holding)
        return blocks

    def get_regions(self):
        return set(region for _, region in self.goal_regions)


def sample_block_poses(blocks, timeout=100):
    block_poses = {}
    for block, region in random_sequence(blocks):
        for _ in irange(0, timeout):
            x = sample_region_pose(region, block)
            if not any(are_colliding(block, x, block2, x2) for block2, x2 in block_poses.iteritems()):
                block_poses[block] = x
                break
        else:
            return sample_block_poses(blocks)
    return block_poses


def sample_grasp_problem(block_w=1., block_h=1., env_w=10.):
    env_region = Region('env', 0., env_w)

    blocks = [
        Block('block0', block_w, block_h, 'red'),
    ]

    initial_regions = [
        env_region,
    ]

    initial_config = 0
    initial_holding = None
    initial_poses = sample_block_poses(zip(blocks, initial_regions))

    goal_config = None
    goal_holding = blocks[0]
    goal_poses = []
    goal_regions = []

    return TMPProblem(env_region, initial_config, initial_holding, initial_poses,
                      goal_config, goal_holding, goal_poses, goal_regions)


def sample_tamp_problem(block_w=1., block_h=1.):
    env_region = Region('env', 0., 20.)

    blocks = [
        Block('block0', block_w, block_h, 'red'),
        Block('block1', block_w, block_h, 'blue'),
    ]

    region = Region('goal', 9., 2.)
    initial_regions = [
        env_region,
        region,
    ]

    initial_config = 0
    initial_holding = None

    initial_poses = sample_block_poses(zip(blocks, initial_regions))

    goal_config = None
    goal_holding = None
    goal_poses = [

    ]
    goal_regions = [
        (blocks[0], region),
    ]

    return TMPProblem(env_region, initial_config, initial_holding, initial_poses,
                      goal_config, goal_holding, goal_poses, goal_regions)
