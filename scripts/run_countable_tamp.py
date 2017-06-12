#!/usr/bin/env python

from time import time

from stripstream.algorithms.plan import get_states
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
#from stripstream.algorithms.focused.focused_planner import focused_planner
#from stripstream.algorithms.hierarchy.replan import replan_hierarchy, first_selector
#from stripstream.algorithms.experimental.state_space import progression
from stripstream.algorithms.search.bfs import get_bfs
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.utils import SEPARATOR
from stripstream.algorithms.utils import DEFAULT, BFS, FAST_DOWNWARD, INCREMENTAL, \
    FOCUSED, LAPKT, FAST_FORWARD, PYPLANNERS, DEFAULT_SEARCH
from stripstream.algorithms.search.lapkt import get_lakpt
from stripstream.algorithms.search.fast_forward import get_fast_forward

#from stripstream.algorithms.search.pyplanners import get_pyplanners

from stripstream.pddl.examples.countable_tamp.countable_tamp_utils import get_grasp_problem, \
    get_distract_problem, get_invert_problem, get_shift_problem, NUM_POSES
from stripstream.pddl.examples.countable_tamp.countable_tamp import compile_problem, visualize_atoms
#from stripstream.pddl.examples.countable_tamp.old_countable_tamp import compile_problem, visualize_atoms
#from stripstream.pddl.examples.countable_tamp.hierarchical_countable_tamp import compile_problem, visualize_atoms, NUM_POSES

import argparse

# TODO - do a version of this without axioms
# TODO - do a version of this which includes objects in actions
# NOTE - need to handle conditional effects as conditions...
# Alternatively can just compile out axioms as currently doing and compare


def solve_countable_tamp(planner, search, visualize, display, verbose=False):
    # tamp_problem = get_grasp_problem(p=3) # 10 | 1000
    # tamp_problem = get_distract_problem(p=3) # 10 | 1000
    #tamp_problem = get_invert_problem(p=3)
    tamp_problem = get_shift_problem(p=3)
    #tamp_problem = get_distract_place_problem(p=40)

    # stream_problem = compile_problem(tamp_problem,
    # stream_fn=generative_streams) # constant_streams | implicit_streams |
    # constant_streams
    stream_problem = compile_problem(tamp_problem)
    print stream_problem
    #stream_problem.operators = stream_problem.convert_axioms_to_effects()
    # stream_problem.replace_axioms()
    # for action in stream_problem.operators:
    #  print action.pddl(True)

    viewer = None
    if visualize:
        from stripstream.pddl.examples.countable_tamp.countable_tamp_viewer import CountableTMPViewer
        viewer = CountableTMPViewer(1, NUM_POSES, title='State', height=250)
        visualize_atoms(viewer, stream_problem.initial_atoms)
        raw_input('Continue?')

    verbose_search = False
    if search == DEFAULT:
        search_fn = DEFAULT_SEARCH
    elif search == BFS:
        search_fn = get_bfs()
    elif search == FAST_DOWNWARD:
        # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
        search_fn = get_fast_downward('eager', verbose=verbose_search)
    elif search == FAST_FORWARD:
        search_fn = get_fast_forward(verbose=verbose_search)
    elif search == LAPKT:
        stream_problem.replace_axioms()  # TODO - can I automate this process?
        search_fn = get_lakpt(verbose=verbose_search)
    # elif search == PYPLANNERS:
    #  search_fn = get_pyplanners(verbose=False)
    else:
        raise ValueError(search)

    t0 = time()
    if planner == INCREMENTAL:
        plan, universe = incremental_planner(
            stream_problem, search=search_fn, frequency=1, verbose=verbose)  # 1 | 20 | 100 | INF
    elif planner == FOCUSED:
        #plan, universe = focused_planner(stream_problem, search=search_fn, greedy=False, verbose=verbose)
        plan, universe = simple_focused(
            stream_problem, search=search_fn, greedy=True, dfs=True, verbose=verbose)
    # elif planner == 'hierarchy':
    #  selector = first_selector # first_selector | all_selector
    #  plan, universe = replan_hierarchy(stream_problem, selector=selector, execute=True, verbose=verbose)
    # elif planner == 'simultaneous':
    #  plan, universe = simultaneous(stream_problem, frequency=frequency, max_time=max_time)
    # elif planner == 'progression':
    #  plan, universe = progression(stream_problem, max_time=max_time)
    else:
        raise ValueError(planner)

    print SEPARATOR
    print 'Planner:', planner
    print 'Search:', search
    print 'Plan:', convert_plan(plan)
    print 'Solved:', plan is not None
    print 'Length:', len(plan) if plan is not None else None
    print 'Time:', time() - t0

    # universe.print_statistics()
    # universe.print_domain_statistics()
    # print SEPARATOR
    #print_plan_stats(plan, universe)

    if display and plan is not None:
        if viewer is None:
            # TODO - unify this across the two
            from stripstream.pddl.examples.countable_tamp.countable_tamp_viewer import CountableTMPViewer
            viewer = CountableTMPViewer(
                1, NUM_POSES, title='State', height=250)
        print '\nExecuting'
        states = get_states(universe, plan)
        for i, state in enumerate(states):
            viewer.clear()
            viewer.draw_environment()
            visualize_atoms(viewer, state)
            raw_input('%s) %s?' %
                      (i, 'Continue' if i != len(states) - 1 else 'Finish'))
    elif viewer is not None:
        raw_input('\nFinish?')

##################################################


def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('--search', help='problem name.', default=DEFAULT)
    parser.add_argument('-focus', action='store_true', help='focused.')
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    parser.add_argument('-display', action='store_true',
                        help='display solution.')
    args = parser.parse_args()

    planner = FOCUSED if args.focus else INCREMENTAL
    solve_countable_tamp(planner, args.search.lower(),
                         args.viewer, args.display)

    #f = lambda: solve_countable_tamp(planner, search, args.viewer, args.display)
    #from misc.profiling import run_profile, str_profile
    #_, prof = run_profile(f)
    # print str_profile(prof)

if __name__ == '__main__':
    main()
