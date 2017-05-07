from time import time
import sys
import os

from stripstream.utils import write, read, INF, SEPARATOR, ensure_dir, safe_remove, remove_dir
from stripstream.algorithms.search.utils import TEMP_DIRECTORY, DOMAIN_PATH, PROBLEM_PATH


ENV_VAR = 'FD_PATH'
FD_BIN = 'bin'

TRANSLATE_DIR = 'translate/'
SEARCH_COMMAND = 'downward %s < '

TRANSLATE_OUTPUT = 'output.sas'
PREPROCESS_OUTPUT = 'output'
SEARCH_OUTPUT = 'sas_plan'


search_options = {
    'dijkstra': '--heuristic "h=blind(transform=adapt_costs(cost_type=PLUSONE))" --search "astar(h,max_time=%s,bound=%s)"',
    'astar': '--heuristic "h=hmax(transform=adapt_costs(cost_type=PLUSONE))" --search "astar(h,max_time=%s,bound=%s)"',
    'wastar1': '--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=true,lm_cost_type=plusone),transform=adapt_costs(cost_type=PLUSONE))" '
    '--search "lazy_wastar([hff,hlm],preferred=[hff,hlm],w=1,max_time=%s,bound=%s)"',
    'wastar2': '--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=true,lm_cost_type=plusone),transform=adapt_costs(cost_type=PLUSONE))" '
    '--search "lazy_wastar([hff,hlm],preferred=[hff,hlm],w=2,max_time=%s,bound=%s)"',
    'wastar3': '--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=true,lm_cost_type=plusone),transform=adapt_costs(cost_type=PLUSONE))" '
    '--search "lazy_wastar([hff,hlm],preferred=[hff,hlm],w=3,max_time=%s,bound=%s)"',


    'eager': '--heuristic "hff=ff(transform=adapt_costs(cost_type=PLUSONE))" '
    '--search "eager_greedy(hff,preferred=hff,max_time=%s,bound=%s)"',
    'lazy': '--heuristic "hlm,hff=lm_ff_syn(lm_rhw(reasonable_orders=true,lm_cost_type=plusone),transform=adapt_costs(cost_type=PLUSONE))" '
    '--search "lazy_greedy([hff,hlm],preferred=[hff,hlm],max_time=%s,bound=%s)"',
}


def has_fd():
    return ENV_VAR in os.environ


def get_fd_root():
    if not has_fd():
        raise RuntimeError('Environment variable %s is not defined.' % ENV_VAR)
    return os.environ[ENV_VAR]


def run_translate(verbose, translate_flags=tuple()):
    t0 = time()

    fd_root = get_fd_root()
    translate_path = os.path.join(fd_root, FD_BIN, TRANSLATE_DIR)
    if translate_path not in sys.path:
        sys.path.append(translate_path)

    temp_argv = sys.argv[:]
    sys.argv = sys.argv[:1] + \
        list(translate_flags) + [DOMAIN_PATH, PROBLEM_PATH]
    import translate
    sys.argv = temp_argv

    if verbose:
        print SEPARATOR
        translate.main()
        print
        print 'from translate import main', time() - t0
    else:
        with open(os.devnull, 'w') as devnull:
            old_stdout = sys.stdout
            sys.stdout = devnull
            try:
                translate.main()
            finally:
                sys.stdout = old_stdout


def run_search(planner, max_time, max_cost, verbose):
    fd_root = get_fd_root()
    search = os.path.join(fd_root, FD_BIN, SEARCH_COMMAND) + TRANSLATE_OUTPUT
    planner_config = search_options[planner] % (max_time, max_cost)
    command = search % planner_config

    t0 = time()
    p = os.popen(command)
    output = p.read()
    if verbose:
        print
        print output
        print command, time() - t0
    if not os.path.exists(SEARCH_OUTPUT):
        return None
    return read(SEARCH_OUTPUT)


def run_fastdownard(planner, max_time, max_cost, verbose):
    if isinstance(max_time, float):
        max_time = int(max_time)
    if max_cost == INF:
        max_cost = 'infinity'
    elif isinstance(max_cost, float):
        max_cost = int(max_cost)
    safe_remove(SEARCH_OUTPUT)
    run_translate(verbose)
    return run_search(planner, max_time, max_cost, verbose)


def lookup_action(universe, action, parameters):
    return universe.get_action(action), tuple(universe.get_object(param) for param in parameters)


def convert_solution(solution, universe):
    lines = solution.split('\n')[:-2]
    plan = []
    for line in lines:
        entries = line.strip('( )').split(' ')
        plan.append(lookup_action(universe, entries[0], entries[1:]))
    return plan


def fast_downward(universe, planner, max_time, max_cost, verbose):
    ensure_dir(TEMP_DIRECTORY)
    write(DOMAIN_PATH, universe.domain_pddl(True, True))
    write(PROBLEM_PATH, universe.problem_pddl(True))

    solution = run_fastdownard(planner, max_time, max_cost, verbose)
    if solution is None:
        return None
    if not verbose:
        remove_dir(TEMP_DIRECTORY)
        safe_remove(TRANSLATE_OUTPUT)
        safe_remove(PREPROCESS_OUTPUT)
        safe_remove(SEARCH_OUTPUT)
    return convert_solution(solution, universe)


def get_fast_downward(config='wastar2', max_time=30, max_cost=INF, verbose=False):
    """
    Returns a FastDownward (FD) search function configured using the arguments.

    :param config: a string that is a key of :data:`.search_options` giving the FD configuration.
    :param max_time: a numeric constant for the maximum FD search time.
    :param max_cost: a numeric constant for the maximum FD solution cost.
    :param verbose: a boolean flag toggling the amount of terminal output
    :return: function wrapper around :func:`.fast_downward`
    """
    return lambda p, mt, mc: fast_downward(p, config, min(mt, max_time), min(mc, max_cost), verbose)
