#!/usr/bin/env python

from time import time
#from stripstream.utils import set_deterministic
from stripstream.algorithms.plan import get_states
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.search.bfs import get_bfs
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.algorithms.utils import BFS, FAST_DOWNWARD, INCREMENTAL, FOCUSED
from stripstream.utils import SEPARATOR

# from stripstream.pddl.examples.continuous_tamp.old_continuous_tamp import compile_problem, \
#  visualize_initial, visualize_goal, visualize_atoms, generative_streams, constant_streams, implicit_streams
from stripstream.pddl.examples.continuous_tamp.continuous_tamp_utils import sample_tamp_problem
from stripstream.pddl.examples.continuous_tamp.continuous_tamp import compile_problem, \
    visualize_atoms, visualize_initial

import argparse


def solve_continuous_tamp(planner, search, visualize, display, verbose=False, deterministic=False):
    # if deterministic:
    #  set_deterministic()

    # sample_tamp_problem | sample_grasp_problem | sample_namo_problem
    tamp_problem = sample_tamp_problem()

    # stream_problem = compile_problem(tamp_problem,
    # streams_fn=generative_streams) # constant_streams | implicit_streams |
    # generative_streams
    stream_problem = compile_problem(tamp_problem)
    print stream_problem
    viewer = None
    if visualize:
        viewer = visualize_initial(tamp_problem, stream_problem)
        raw_input('Continue?')
        # viewer.save('initial')

    if search == BFS:
        search_fn = get_bfs()
    elif search == FAST_DOWNWARD:
        # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
        search_fn = get_fast_downward('eager')
    else:
        raise ValueError(search)

    t0 = time()
    if planner == INCREMENTAL:
        plan, universe = incremental_planner(
            stream_problem, search=search_fn, frequency=1, verbose=verbose)  # 1 | 20 | 100 | INF
    elif planner == FOCUSED:
        #plan, universe = focused_planner(stream_problem, search=search_fn, greedy=False, verbose=verbose)
        plan, universe = simple_focused(
            stream_problem, search=search_fn, greedy=True, optimal=False, verbose=verbose)
        #plan, universe = plan_focused(stream_problem, search=search_fn, greedy=True, optimal=False, verbose=verbose)
    else:
        raise ValueError(planner)

    print SEPARATOR
    print 'Planner:', planner
    print 'Search:', search
    print 'Plan:', convert_plan(plan)
    print 'Solved:', plan is not None
    print 'Length:', len(plan) if plan is not None else None
    print 'Time:', time() - t0

    # TODO - sometimes the movement actions, especially for the lazy
    # algorithm, screw up in the display for some reason...
    if display and plan is not None:
        if viewer is None:
            viewer = visualize_initial(tamp_problem, stream_problem)
        print '\nExecuting'
        # TODO - need to return the problem
        states = get_states(universe, plan)
        for i, state in enumerate(states):
            viewer.clear_state()
            visualize_atoms(viewer, state)
            raw_input('%s) %s?' %
                      (i, 'Continue' if i != len(states) - 1 else 'Finish'))
    elif viewer is not None:
        raw_input('\nFinish?')

##################################################


def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-fd', action='store_true', help='FastDownward.')
    parser.add_argument('-focus', action='store_true', help='focused.')
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    parser.add_argument('-display', action='store_true',
                        help='display solution.')
    args = parser.parse_args()

    planner = FOCUSED if args.focus else INCREMENTAL
    search = FAST_DOWNWARD if args.fd else BFS
    solve_continuous_tamp(planner, search, args.viewer, args.display)

if __name__ == '__main__':
    main()
