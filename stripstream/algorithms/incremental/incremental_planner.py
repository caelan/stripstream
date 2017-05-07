from collections import deque
from time import time

from stripstream.algorithms.universe import Universe
from stripstream.algorithms.utils import DEFAULT_SEARCH
from stripstream.utils import irange, INF
from stripstream.algorithms.plan import plan_cost, print_plan_stats
from stripstream.pddl.logic.atoms import Atom, Initialize
from stripstream.pddl.objects import Object
from stripstream.pddl.cond_streams import TestCondStream
from stripstream.pddl.utils import get_value, convert_plan


def add_streams(universe, queue):
    for stream in universe.new_streams:
        if stream.cond_stream.eager:
            queue.appendleft(stream)
        else:
            queue.append(stream)
    universe.new_streams = []


def call_stream(universe, stream, **kwargs):
    if universe.verbose:
        print 'Called', stream
    if isinstance(stream.cond_stream, TestCondStream):
        universe.test_calls += 1
    universe.calls += 1
    values = stream.call(universe=universe, **kwargs)
    for value in values:
        if isinstance(value, Object):
            universe.add_object(value)
        elif isinstance(value, Atom):
            universe.add_initial_atom(value)
        elif isinstance(value, Initialize):
            universe.add_perm_initialize(value)
        else:
            raise ValueError(value)
    return values


def call_queue(queue, universe, frequency, max_time, max_calls=INF):
    local_calls = 0
    universe.new_problem = False
    while queue and (not universe.new_problem or local_calls < frequency) and universe.calls < max_calls:
        if time() - universe.start_time > max_time:
            return False
        stream = queue.popleft()
        assert all(
            con in universe.initial_atoms for con in stream.instantiate_conditions())

        call_stream(universe, stream)
        add_streams(universe, queue)
        if not stream.cond_stream.eager or not stream.called:
            local_calls += 1
        if not stream.enumerated:
            queue.append(stream)
        else:
            universe.enumerated += 1
    return universe.new_problem


def call_queue_wave(queue, universe, max_values, max_time, max_calls=INF):
    wave = 0
    universe.new_problem = False
    while queue and (not universe.new_problem or wave < max_values) and universe.calls < max_calls:
        called_queue = deque()
        while queue and universe.calls < max_calls:
            if time() - universe.start_time > max_time:
                return False
            stream = queue.popleft()
            if not all(con in universe.initial_atoms for con in stream.instantiate_conditions()):
                print 'Warning: not all conditions achieved'
                print stream.instantiate_conditions()
                print stream
                called_queue.append(stream)
                continue
            call_stream(universe, stream)
            if not stream.enumerated:
                called_queue.append(stream)
            else:
                universe.enumerated += 1
        add_streams(universe, queue)
        queue.extend(called_queue)
        wave += 1

    return universe.new_problem


def print_status(universe):
    print 'Iteration: %d | Time: %s | Calls: %s | Search Time: %s' % (universe.iterations,
                                                                      round(time() - universe.start_time, 3), universe.calls, round(universe.search_time, 3))


def incremental_planner(problem, search=DEFAULT_SEARCH, max_time=INF, max_iterations=INF, max_calls=INF,
                        waves=True, frequency=1, optimal=False,
                        verbose=False, debug=False):
    """
    Incremental algorithm.

    :param problem: :class:`.STRIPStreamProblem`
    :param search: python function of the search subroutine
    :param max_time: numeric maximum planning time
    :param max_iterations: int maximum subroutine calls
    :param waves: boolean flag when ``True`` considers each stream evaluation iteration to be the current queue rather than a single stream
    :param frequency: numeric number of evaluation iterations given by ``waves`` (``float('inf')`` implies to evaluate all streams per iteration)
    :param optimal: boolean flag which saves the best current solution
    :param verbose: boolean flag which toggles the print output
    :param debug: boolean flag which prints the objects on each iteration
    :return: a sequence of :class:`.Action` and tuple of :class:`.Constant` pairs as well as :class:`.Universe`
    """

    universe = Universe(problem, use_ground=False, verbose=verbose)

    queue = deque()
    add_streams(universe, queue)
    best_plan, best_cost = None, INF
    for _ in irange(0, max_iterations):
        print 'Iteration: %d | Time: %s | Calls: %s | Search Time: %s | Solved: %s | Cost: %s' % (universe.iterations,
                                                                                                  round(
                                                                                                      time() - universe.start_time, 3), universe.calls, round(universe.search_time, 3),
                                                                                                  best_plan is not None, best_cost)
        if debug:
            for type in universe.type_to_objects:
                print type, len(universe.type_to_objects[type]), map(get_value, universe.type_to_objects[type])[:10]
            raw_input('Continue?\n')
        universe.iterations += 1
        t0 = time()
        plan = search(universe, max_time - (time() -
                                            universe.start_time), best_cost)
        universe.search_time += time() - t0
        if plan is not None:
            cost = plan_cost(universe, plan)
            if cost < best_cost:
                best_plan, best_cost = plan, cost
                if verbose:
                    print best_cost, convert_plan(best_plan)
            if not optimal:
                break
        if waves and not call_queue_wave(queue, universe, frequency, max_time, max_calls):
            break
        if not waves and not call_queue(queue, universe, frequency, max_time, max_calls):
            break
    if verbose:
        universe.print_statistics()
        if best_plan is not None:
            print_plan_stats(best_plan, universe)
            print 'Cost:', best_cost
            print 'Length:', len(best_plan)

    return best_plan, universe
