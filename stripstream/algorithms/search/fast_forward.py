
import os
import re

from stripstream.utils import write, ensure_dir, remove_dir
from stripstream.algorithms.search.utils import TEMP_DIRECTORY, DOMAIN_PATH, PROBLEM_PATH
from stripstream.algorithms.search.fast_downward import lookup_action


ENV_VAR = 'FF_PATH'
SEARCH_COMMAND = 'ff -o %s -f %s'


def has_ff():
    return ENV_VAR in os.environ


def get_ff_root():
    if not has_ff():
        raise RuntimeError('Environment variable %s is not defined.' % ENV_VAR)
    return os.environ[ENV_VAR]


def fast_forward(universe, derived, verbose):
    ensure_dir(TEMP_DIRECTORY)
    write(DOMAIN_PATH, universe.domain_pddl(False, derived))
    write(PROBLEM_PATH, universe.problem_pddl(False))

    command = os.path.join(get_ff_root(), SEARCH_COMMAND %
                           (DOMAIN_PATH, PROBLEM_PATH))
    p = os.popen(command)
    output = p.read()
    if verbose:
        print output
        print command
    if not verbose:
        remove_dir(TEMP_DIRECTORY)

    success = 'ff: found legal plan as follows'
    index = output.find(success)
    if index == -1:
        return None
    plan = []
    for line in re.findall('\d+:(?: \w+)+\n', output):
        entries = re.findall(' (\w+)', line.lower())
        plan.append(lookup_action(universe, entries[0], entries[1:]))
    return plan


def get_fast_forward(derived=True, verbose=False):
    """
    Returns a FastDownward (FD) search function configured using the arguments.

    :param derived: a boolean flag toggling the use of derived predicates
    :param verbose: a boolean flag toggling the amount of terminal output
    :return: function wrapper around :func:`.fast_downward`
    """
    return lambda p, mt, mc: fast_forward(p, derived, verbose)
