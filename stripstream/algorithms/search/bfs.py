from stripstream.utils import INF
from stripstream.pddl.logic.atoms import Atom
from stripstream.algorithms.instantiation import instantiate_actions, instantiate_axioms
from collections import deque, namedtuple
from time import time


def retrace(fluents, state_space):
    if state_space[fluents].parent is None:
        return []
    return retrace(state_space[fluents].parent, state_space) + [state_space[fluents].action]


def bfs(universe, max_time, max_cost, verbose):
    Node = namedtuple('Node', ['cost', 'action', 'parent'])

    start_time = time()

    action_instances = list(instantiate_actions(universe))

    axiom_instances = list(instantiate_axioms(universe))

    static_atoms = {atom for atom in universe.get_initial_atoms() if isinstance(atom, Atom) and
                    atom.predicate not in universe.fluent_predicates and atom.predicate not in universe.derived_predicates}
    initial_fluents = frozenset(universe.initial_fluents())
    state_space = {initial_fluents: Node(0, None, None)}
    queue = deque([initial_fluents])
    while len(queue) > 0 and time() - start_time < max_time:
        current_fluents = queue.popleft()
        node = state_space[current_fluents]

        applied_axioms = set()
        state = static_atoms | current_fluents
        new_derived = True
        while new_derived and time() - start_time < max_time:
            new_derived = False
            for axiom in axiom_instances:
                if axiom not in applied_axioms and axiom.is_applicable(state, universe.type_to_objects):
                    state = axiom.apply(state, universe.type_to_objects)
                    applied_axioms.add(axiom)
                    new_derived = True

        if universe.goal_formula.holds(state, universe.type_to_objects):
            return map(lambda a: (a.lifted, a.args), retrace(current_fluents, state_space))

        for action in action_instances:
            if node.cost + action.cost <= max_cost and action.is_applicable(state, universe.type_to_objects):
                next_fluents = frozenset(action.apply(
                    current_fluents, universe.type_to_objects))
                if next_fluents not in state_space:
                    state_space[next_fluents] = Node(
                        node.cost + 1, action, current_fluents)
                    queue.append(next_fluents)
    return None


def get_bfs(max_time=30, max_cost=INF, verbose=False):
    """
    Returns a breadth-first search (BFS) search function configured using the arguments.

    :param max_time: a numeric constant for the maximum BFS search time.
    :param max_cost: a numeric constant for the maximum BFS solution cost.
    :param verbose: a boolean flag toggling the amount of terminal output
    :return: function wrapper around :class:`.bfs`
    """
    return lambda p, mt, mc: bfs(p, min(mt, max_time), min(mc, max_cost), verbose)
