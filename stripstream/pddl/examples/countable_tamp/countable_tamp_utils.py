from stripstream.utils import INF

NUM_POSES = INF


class TMPProblem(object):

    def __init__(self, initial_config, initial_holding, initial_poses,
                 goal_config, goal_holding, goal_poses):
        self.initial_config = initial_config
        self.initial_holding = initial_holding
        self.initial_poses = initial_poses
        self.goal_config = goal_config
        self.goal_holding = goal_holding
        self.goal_poses = goal_poses

    def get_blocks(self):
        blocks = self.initial_poses.keys()
        if self.initial_holding is not False:
            blocks.append(self.initial_holding)
        return blocks


def get_grasp_problem(p=100):
    initial_config = 0
    initial_holding = False
    initial_poses = {'block1': p}
    goal_config = None
    goal_holding = 'block1'
    goal_poses = {}
    return TMPProblem(initial_config, initial_holding, initial_poses, goal_config, goal_holding, goal_poses)


def get_distract_problem(p=100):
    initial_config = 0
    initial_holding = False
    initial_poses = [('block%s' % p, p) for p in range(1, p + 1)]
    goal_config = None
    goal_holding = initial_poses[-1][0]
    goal_poses = {}
    return TMPProblem(initial_config, initial_holding, dict(initial_poses), goal_config, goal_holding, goal_poses)


def get_invert_problem(p=3):
    blocks = ['block%s' % p for p in range(0, p)]
    initial_config = 0
    initial_holding = False
    initial_poses = [(block, p) for p, block in enumerate(blocks)]
    goal_config = None
    goal_holding = None
    goal_poses = {block: len(blocks) - p - 1 for p, block in enumerate(blocks)}
    return TMPProblem(initial_config, initial_holding, dict(initial_poses), goal_config, goal_holding, goal_poses)


def get_shift_problem(p=3):
    blocks = ['block%s' % p for p in range(0, p)]
    initial_config = 0
    initial_holding = False
    initial_poses = [(block, p) for p, block in enumerate(blocks)]
    goal_config = None
    goal_holding = None
    goal_poses = {block: p + 1 for p, block in enumerate(blocks)}
    return TMPProblem(initial_config, initial_holding, dict(initial_poses), goal_config, goal_holding, goal_poses)


def get_distract_place_problem(p=100):
    initial_config = 0
    initial_holding = False
    initial_poses = [('block%s' % p, p) for p in range(1, p + 1)]
    goal_config = None
    goal_holding = None
    goal_poses = {'block1': 0}
    return TMPProblem(initial_config, initial_holding, dict(initial_poses), goal_config, goal_holding, goal_poses)
