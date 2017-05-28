
import os

from stripstream.utils import write, read, ensure_dir, safe_remove, remove_dir
from stripstream.algorithms.search.utils import TEMP_DIRECTORY, DOMAIN_PATH, PROBLEM_PATH
from stripstream.algorithms.search.fast_downward import lookup_action


ENV_VAR = 'LAPKT_PATH'

LAPKT_PLANNERS = {

    'agl_bfs_f': 'agile/seq-agl-bfs-f/bfs_f',
    'siw': 'agile/seq-agl-siw/siw',
    'sat_bfs_f': 'agile/seq-sat-bfs-f/at_bfs_f',
}

COMMAND = ' --domain %s --problem %s --output %s'
PLANNER_OUTPUT = 'sas_plan'
PLANNER_DETAILS = 'execution.details'


def has_lapkt():
    return ENV_VAR in os.environ


def get_lapkt_root():
    if not has_lapkt():
        raise RuntimeError('Environment variable %s is not defined.' % ENV_VAR)
    return os.environ[ENV_VAR]


def lapkt(universe, search, verbose):
    ensure_dir(TEMP_DIRECTORY)
    write(DOMAIN_PATH, universe.domain_pddl(True, False))
    write(PROBLEM_PATH, universe.problem_pddl(True))
    safe_remove(PLANNER_OUTPUT)

    planner_path = os.path.join(get_lapkt_root(), LAPKT_PLANNERS[search])
    command = planner_path + \
        COMMAND % (DOMAIN_PATH, PROBLEM_PATH, PLANNER_OUTPUT)
    if verbose:
        print command
    p = os.popen(command)
    output = p.read()
    if verbose:
        print output

    SUCCESS = 'Plan found'
    if output.find(SUCCESS) == -1:
        return None
    plan = []
    for line in read(PLANNER_OUTPUT).split('\n')[:-1]:
        entries = line.strip('()').lower().split(' ')
        plan.append(lookup_action(universe, entries[0], entries[1:]))
    if not verbose:
        remove_dir(TEMP_DIRECTORY)
        safe_remove(PLANNER_OUTPUT)
        safe_remove(PLANNER_DETAILS)
    return plan


def get_lakpt(search='agl_bfs_f', verbose=False):
    """
    Returns a FastDownward (FD) search function configured using the arguments.

    :param search: name of a LAPKT search algorithm
    :param verbose: a boolean flag toggling the amount of terminal output
    :return: function wrapper around :func:`.fast_downward`
    """
    return lambda p, mt, mc: lapkt(p, search, verbose)
