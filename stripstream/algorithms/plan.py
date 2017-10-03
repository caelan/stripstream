from collections import defaultdict
from stripstream.algorithms.instantiation import instantiate_axioms
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.predicates import TotalCost
from stripstream.pddl.logic.utils import get_increases
from stripstream.utils import INF, SEPARATOR


def plan_length(universe, plan):
    if plan is None:
        return INF
    return len(plan)


def plan_cost(universe, plan):
    if plan is None:
        return INF
    cost = universe.initial_cost
    for action, args in plan:
        instance = action.instantiate(args)
        increases = get_increases(instance.effect)

        new_cost = 0
        if action.cost is not None:
            new_cost += action.cost
        for increase in increases:
            if increase.function == TotalCost():
                if isinstance(increase.value, Atom):
                    if increase.value in universe.perm_functions:
                        value = universe.perm_functions[increase.value]
                    elif increase.value in universe.temp_functions:
                        value = universe.temp_functions[increase.value]
                    else:
                        raise ValueError(increase)
                else:
                    value = increase.value
                new_cost += value

        cost += new_cost
    return cost


def substitute_axioms(condition, state, universe):
    if condition.holds(state, universe.type_to_objects):
        return
    augmented = False
    for atom in condition.get_atoms():
        if universe.is_derived(atom):
            axiom = universe.derived_predicates[atom.predicate]
            condition.substitute(axiom.effect, axiom.condition)
            augmented = True
    if augmented:
        substitute_axioms(condition, state, universe)


def apply_axioms(state, constants, axioms):

    unused_axioms = set(axioms)
    while True:
        terminate = True
        for axiom in list(unused_axioms):
            if axiom.is_applicable(state, constants):
                state = axiom.apply(state, constants)
                terminate = False
                unused_axioms.remove(axiom)
        if terminate:
            break
    return state


def get_states(universe, plan):
    instances = [operator.instantiate(parameters)
                 for operator, parameters in plan]
    constants = universe.type_to_objects
    axioms = list(instantiate_axioms(universe))
    state = universe.initial_atoms
    yield apply_axioms(state, constants, axioms)
    for i, instance in enumerate(instances):

        state = instance.apply(state, constants)
        yield apply_axioms(state, constants, axioms)


def feasible_subplan(universe, plan):
    instances = [action.instantiate(args) for action, args in plan]
    conditions = [instance.condition for instance in instances]
    state = universe.get_initial_atoms()
    subplan = []
    for i, (condition, instance) in enumerate(zip(conditions, instances) + [(universe.goal_formula, None)]):
        substitute_axioms(condition, state, universe)
        if not condition.holds(state, universe.type_to_objects):
            return subplan, False
        if instance is not None:
            state = instance.apply(state, universe.type_to_objects)
            subplan.append(plan[i])
    return subplan, True


def is_solution(universe, plan):
    instances = [action.instantiate(args) for action, args in plan]
    conditions = [instance.condition for instance in instances]
    state = universe.get_initial_atoms()
    for condition, instance in zip(conditions, instances) + [(universe.goal_formula, None)]:
        substitute_axioms(condition, state, universe)
        if not condition.holds(state, universe.type_to_objects):
            return False
        if instance is not None:
            state = instance.apply(state, universe.type_to_objects)
    return True


def supporting_static_predicates(plan, universe):
    supporting_atoms = set()
    instances = [action.instantiate(args) for action, args in plan]
    conditions = [instance.condition for instance in instances]
    state = universe.get_initial_atoms()
    for condition, instance in zip(conditions, instances) + [(universe.goal_formula, None)]:
        substitute_axioms(condition, state, universe)
        assert condition.holds(state, universe.type_to_objects)
        for condition in condition.positive_supporters(state, universe.type_to_objects):
            if condition.predicate in universe.stream_predicates:
                supporting_atoms.add(condition)
        if instance is not None:
            state = instance.apply(state, universe.type_to_objects)
    return supporting_atoms


def print_plan_stats(plan, universe):
    print 'Plan statistics'
    print plan
    stream_calls = defaultdict(float)
    stream_time = defaultdict(float)
    values = {obj for _, args in plan for obj in args} | supporting_static_predicates(
        plan, universe)
    for value in values:
        call_stream, calls = universe.get_min_attr(value, 'prior_calls')
        if call_stream is not None:
            stream_calls[call_stream] = max(stream_calls[call_stream], calls)
        time_stream, time = universe.get_min_attr(value, 'prior_time')
        if time_stream is not None:
            stream_time[time_stream] = max(stream_time[time_stream], time)

    cs_calls = defaultdict(float)
    cs_time = defaultdict(float)
    for stream in stream_calls:
        cs_calls[stream.cond_stream] += stream_calls[stream]
        cs_time[stream.cond_stream] += stream_time[stream]

    for cs in cs_calls:
        print cs, cs_calls[cs], cs_time[cs]
    print SEPARATOR
