from time import time
from collections import deque
from collections import defaultdict, namedtuple
from heapq import heappush, heappop

from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.operators import ActionInstance
from stripstream.utils import irange, INF, set_union
from stripstream.algorithms.instantiation import parameter_product2
from stripstream.algorithms.universe import Universe
from stripstream.algorithms.focused.utils import AbstractConstant, make_abstract_constant, Concrete, is_concrete,  constant_from_concrete, has_abstract_inputs, replace_abstract_constants, partition_values, get_target_atoms,  get_stream_functions, add_stream_cost
from stripstream.algorithms.incremental.incremental_planner import DEFAULT_SEARCH, print_status, call_stream
from stripstream.pddl.utils import get_value, convert_plan
from stripstream.algorithms.plan import plan_cost, substitute_axioms
from stripstream.pddl.objects import Object
from stripstream.priority_queue import PriorityQueue


def focused_debug(universe):
    for type in universe.type_to_objects:
        print type, len(universe.type_to_objects[type]),      map(get_value, filter(lambda o: not isinstance(o, AbstractConstant), universe.type_to_objects[type]))[:10]
    raw_input('Continue?\n')


def abstract_focused_debug(universe):
    print 'Streams:', len(universe.new_streams)
    for type, constants in universe.type_to_objects.iteritems():
        print type, len(constants), constants
    raw_input('Continue?\n')


def is_blocked(stream, universe):
    return stream in universe.perm_blocked or stream in universe.temp_blocked


def block_stream(stream, universe):
    if stream.enumerated:
        universe.perm_blocked.add(stream)
        universe.enumerated += 1
    else:
        universe.temp_blocked.add(stream)


def is_real_stream(stream, universe):
    return all(inp in universe.real_objects for inp in stream.inputs) and all(con in universe.real_initial_atoms for con in stream.conditions)


def call_real_stream(stream, universe, **kwargs):
    assert is_real_stream(stream, universe)
    if stream.enumerated:

        return []
    values = call_stream(universe, stream, **kwargs)
    if stream.enumerated:
        universe.enumerated += 1
    for value in values:
        if isinstance(value, Atom) and value not in values:
            values += value.args
    for value in values:
        if isinstance(value, Atom):
            universe.real_initial_atoms.add(value)
            assert not any(isinstance(arg, AbstractConstant)
                           for arg in value.args)
            universe.real_objects.update(value.args)
        elif isinstance(value, Object):
            assert not isinstance(value, AbstractConstant)
            universe.real_objects.add(value)
    block_stream(stream, universe)
    return values


def call_abstract_stream(universe, stream, max_level):
    if stream.cond_stream.max_level is not None:
        max_level = stream.cond_stream.max_level
    stream.include = False
    if stream.cond_stream.eager and not has_abstract_inputs(stream):
        if not stream.enumerated:
            call_real_stream(stream, universe)
        assert stream.enumerated
        return
    if not stream.cond_stream.plannable or stream.cond_stream.cost == INF:
        return
    stream.include = True
    level = max([0] + [inp.level if isinstance(inp, AbstractConstant)
                       else 0 for inp in stream.inputs])
    if level <= max_level:
        outputs = [make_abstract_constant(out.type)
                   for out in stream.cond_stream.outputs]
        for out in outputs:
            out.level = level + 1
            out.stream = stream
        stream.shared = False
    else:
        outputs = stream.cond_stream.abs_outputs
        stream.shared = True
    stream.level = level
    stream.abs_outputs = outputs
    stream.abs_effects = stream.cond_stream.instantiate_effects(
        stream.inputs, outputs)
    for out in outputs:
        universe.add_object(out)
    for atom in stream.abs_effects:
        if isinstance(atom, Atom):
            universe.add_initial_atom(atom)

        else:
            raise ValueError(atom)


def add_streams(universe):
    for i in range(universe.queue_index, len(universe.new_streams)):
        universe.queue.append(universe.new_streams[i])
    universe.queue_index = len(universe.new_streams)


def make_streams(universe, max_level, max_time=INF, frequency=INF):

    add_streams(universe)
    local_calls = 0
    universe.new_problem = False
    while universe.queue and (not universe.new_problem or local_calls < frequency):
        if time() - universe.start_time > max_time:
            return False
        stream = universe.queue.popleft()
        assert all(
            con in universe.initial_atoms for con in stream.instantiate_conditions())
        call_abstract_stream(universe, stream, max_level)
        add_streams(universe)
        if not stream.cond_stream.eager or not stream.called:
            local_calls += 1


AtomN = namedtuple('AtomN', ['cost', 'level', 'stream'])
StreamN = namedtuple('StreamN', ['cost', 'level'])


def determine_reachable(universe, op=sum, unit=False):
    state = universe.real_initial_atoms
    state.update(map(Concrete, universe.real_objects))
    state.add(None)
    unprocessed = defaultdict(list)
    for stream in universe.new_streams:
        if stream.include and not is_blocked(stream, universe):
            stream._conditions = stream.conditions + \
                [Concrete(inp) for inp in stream.inputs] + [None]
            stream._effects = stream.abs_effects + \
                [Concrete(out) for out in stream.abs_outputs]
            stream._remaining = len(stream._conditions)
            for atom in stream._conditions:
                unprocessed[atom].append(stream)

    atom_nodes = {literal: AtomN(0, 0, None) for literal in state}
    queue = [(pair.cost, atom) for atom, pair in atom_nodes.iteritems()]
    while queue:
        _, atom = heappop(queue)
        if atom not in unprocessed:
            continue
        for stream in unprocessed[atom]:
            stream._remaining -= 1
            if stream._remaining == 0:
                stream._cost = op(atom_nodes[con].cost for con in stream._conditions) + (
                    stream.cond_stream.cost if not unit else 1)
                stream._level = max(
                    atom_nodes[con].level for con in stream._conditions) + 1
                for effect in stream._effects:
                    if effect not in atom_nodes or stream._cost < atom_nodes[effect].cost:
                        atom_nodes[effect] = AtomN(
                            stream._cost, stream._level, stream)
                        heappush(queue, (stream._cost, effect))
        del unprocessed[atom]
    del atom_nodes[None]
    return atom_nodes


def extract_streams(atom_nodes, goals):
    new_goals = goals.copy()
    streams = set()
    orders = set()
    queue = deque(new_goals)
    while queue:
        goal = queue.popleft()
        stream = atom_nodes[goal].stream
        if stream is not None:
            if stream not in streams:
                stream.targets = set()
                streams.add(stream)
                for atom in stream._conditions:
                    if atom is not None:
                        if atom_nodes[atom].stream is not None:
                            orders.add((atom_nodes[atom].stream, stream))
                        if atom not in new_goals:
                            new_goals.add(atom)
                            queue.append(atom)
            stream.targets.add(goal)

    return topological_sort(streams, orders)


def topological_sort(streams, edges):
    incoming_edges = defaultdict(set)
    outgoing_edges = defaultdict(set)
    for start, end in edges:
        incoming_edges[end].add(start)
        outgoing_edges[start].add(end)

    priority_fn = lambda stream: stream.cond_stream.order

    ordering = []
    queue = PriorityQueue()
    for stream in streams:
        if not incoming_edges[stream]:
            queue.push(priority_fn(stream), stream)
    while not queue.empty():
        stream1 = queue.pop()
        ordering.append(stream1)
        for stream2 in outgoing_edges[stream1]:
            incoming_edges[stream2].remove(stream1)
            if not incoming_edges[stream2]:
                queue.push(priority_fn(stream2), stream2)
    return ordering


def bind_stream_outputs(stream, inputs, universe, history=True, **kwargs):
    if inputs is None:
        return
    bound_stream = stream.cond_stream(inputs)
    if not is_real_stream(bound_stream, universe):
        return
    values = call_real_stream(bound_stream, universe, **kwargs)
    bound_map = dict(zip(stream.inputs, inputs))
    targets = [effect.instantiate(bound_map) for effect in stream.targets]
    target_atoms, target_objects = partition_values(targets)
    target_parameters = filter(lambda o: isinstance(
        o, AbstractConstant), target_objects)
    produced_atoms, produced_objects = partition_values(values)
    if not target_parameters:
        return iter([{}]) if target_atoms <= produced_atoms else iter([])
    if universe.verbose:
        print
        print bound_stream
        print values
        print target_atoms, target_objects
        print target_parameters
        print produced_atoms, produced_objects

    return parameter_product2(target_parameters, produced_objects, target_atoms, produced_atoms)


def first_bindings(order, universe, shared=True):
    if not order:
        return {}
    for stream in order:
        if (not stream.shared or shared) and not has_abstract_inputs(stream):
            _ = call_real_stream(stream, universe)
            return None
    raise ValueError(order)


def first_level_bindings(order, universe, greedy=True, shared=True):
    if not order:
        return {}
    called = 0
    for stream in order:
        if (not stream.shared or shared) and not has_abstract_inputs(stream):
            _ = call_real_stream(stream, universe)
            called += 1

    assert called
    return None


def single_bindings(order, universe, greedy=True, shared=True):
    failure = False
    bindings = {}
    for stream in order:
        local_failure = True
        if shared or not stream.shared:
            inputs = replace_abstract_constants(stream.inputs, bindings)
            outputs = next(bind_stream_outputs(stream, inputs, universe), None)
            if outputs is not None:
                local_failure = False
                for abs_const, real_const in outputs.iteritems():
                    assert stream.shared or abs_const not in bindings
                    if bindings.get(abs_const, real_const) == real_const:
                        bindings[abs_const] = real_const
                    else:
                        local_failure = True
        failure |= local_failure
        if greedy and failure:
            break
    return bindings if not failure else None


def get_dependent_streams(stream, order, bindings, allow_abstract=False):
    copied_bindings = bindings.copy()
    for abs_out, param_out in zip(stream.abs_outputs, stream.cond_stream.outputs):
        copied_bindings[abs_out] = param_out
    related = set()
    for other in order:
        if stream != other:
            intersect = set(stream.abs_outputs) & set(other.inputs)
            if intersect:
                bound_other = other.cond_stream(
                    map(lambda const: copied_bindings.get(const, const), other.inputs))
                if allow_abstract or all(not isinstance(inp, AbstractConstant) for inp in set(bound_other.inputs) - intersect):
                    related.add(bound_other)

    return related


def bind_atom(atom, bindings):
    return atom.predicate(*map(lambda const: bindings.get(const, const), atom.args))


def get_dependent_goals(stream, order, bindings, allow_abstract=False):
    if not stream.abs_outputs:
        bound_targets = {bind_atom(goal, bindings) for goal in stream.targets}
        return bound_targets

    goals = set_union(other.targets for other in order)
    copied_bindings = bindings.copy()
    for abs_out, param_out in zip(stream.abs_outputs, stream.cond_stream.outputs):
        copied_bindings[abs_out] = param_out
    related = set()
    for goal in goals:
        if not is_concrete(goal):
            intersect = set(stream.abs_outputs) & set(goal.args)
            if intersect:
                bound_goal = bind_atom(goal, copied_bindings)
                if allow_abstract or all(not isinstance(inp, AbstractConstant) for inp in set(bound_goal.args) - intersect):
                    related.add(bound_goal)

    return related


def dfs_bindings(order, universe, bindings={}, greedy=True, shared=True):
    if not order:
        return bindings
    stream = order[0]
    if shared or not stream.shared:
        inputs = replace_abstract_constants(stream.inputs, bindings)

        dependent_atoms = get_dependent_goals(stream, order, bindings)
        outputs_list = list(bind_stream_outputs(
            stream, inputs, universe, dependent_atoms=dependent_atoms))
        for outputs in outputs_list:
            new_bindings = bindings.copy()
            local_failure = False
            for abs_const, real_const in outputs.iteritems():
                assert stream.shared or abs_const not in new_bindings
                if new_bindings.get(abs_const, real_const) == real_const:
                    new_bindings[abs_const] = real_const
                else:
                    local_failure = True
            result_bindings = dfs_bindings(
                order[1:], universe, bindings=new_bindings, greedy=greedy, shared=shared)
            if not local_failure and result_bindings is not None:
                return result_bindings
        if not outputs_list and not greedy:
            dfs_bindings(order[1:], universe, bindings=bindings,
                         greedy=greedy, shared=shared)
    elif not greedy:
        dfs_bindings(order[1:], universe, bindings=bindings,
                     greedy=greedy, shared=shared)
    return None


def is_real_solution(universe, plan):
    type_to_objects = defaultdict(list)
    for obj in universe.real_objects:
        type_to_objects[obj.type].append(obj)
    instances = [action.instantiate(args) for action, args in plan]
    conditions = [instance.condition for instance in instances]
    state = universe.real_initial_atoms
    for condition, instance in zip(conditions, instances) + [(universe.goal_formula, None)]:
        substitute_axioms(condition, state, universe)
        if not condition.holds(state, type_to_objects):
            return False
        if instance is not None:
            state = instance.apply(state, type_to_objects)
    return True


def solve(universe, search, max_time):
    t0 = time()
    plan = search(universe, max_time - (time() - universe.start_time), INF)
    universe.search_time += time() - t0
    return plan


def solve_real(universe, search, max_time):
    temp_initial_atoms = universe.initial_atoms.copy()
    temp_type_to_objects = universe.type_to_objects.copy()
    universe.initial_atoms = universe.real_initial_atoms
    universe.type_to_objects = defaultdict(set)
    for obj in universe.real_objects:
        universe.type_to_objects[obj.type].add(obj)

    plan = solve(universe, search, max_time)
    universe.initial_atoms = temp_initial_atoms
    universe.type_to_objects = temp_type_to_objects
    return plan


def solve_abstract(universe, abstract_atoms, search, max_time, stream_cost):
    temp_initial_atoms = universe.initial_atoms.copy()
    temp_type_to_objects = universe.type_to_objects.copy()

    universe.initial_atoms = {
        atom for atom in abstract_atoms if atom.predicate is not Concrete}
    universe.type_to_objects = defaultdict(set)
    for atom in abstract_atoms:
        if is_concrete(atom):
            arg = constant_from_concrete(atom)
            universe.type_to_objects[arg.type].add(arg)

    plan = solve(universe, search, max_time)
    goals = None
    if plan is not None:
        static_atoms = universe.initial_atoms - universe.real_initial_atoms
        goals = set_union(get_target_atoms(universe, plan, static_atoms))
    universe.initial_atoms = temp_initial_atoms
    universe.type_to_objects = temp_type_to_objects
    return plan, goals


def initialize_universe(universe):
    universe.real_initial_atoms = universe.initial_atoms.copy()
    universe.real_objects = {
        obj for ty in universe.type_to_objects for obj in universe.type_to_objects[ty]}
    for cs in universe.problem.cond_streams:
        outputs = tuple(make_abstract_constant(out.type, shared=True)
                        for out in cs.outputs)
        level = INF
        for out in outputs:
            out.level = level
            out.stream = None
        cs.level = level
        cs.abs_outputs = outputs
    universe.queue = deque()
    universe.queue_index = 0


def produce_bindings(universe, order, plan, greedy, shared, dfs):
    if dfs:
        bindings = dfs_bindings(order, universe, greedy=greedy, shared=shared)
    else:

        bindings = single_bindings(
            order, universe, greedy=greedy, shared=shared)
    if bindings is None:
        return None
    bound_plan = []
    for action, args in plan:
        bound_args = replace_abstract_constants(args, bindings)
        assert bound_args is not None
        bound_plan.append((action, bound_args))
    return bound_plan


def simple_focused(problem, search=DEFAULT_SEARCH, max_time=INF, max_iterations=INF,
                   optimal=False, stream_cost=10, check_feasible=False, max_level=0,
                   greedy=True, shared=True, dfs=True,
                   verbose=False, debug=False):

    universe = Universe(problem, use_ground=False,
                        make_stream_instances=True, make_action_instances=True)

    initialize_universe(universe)
    universe.action_to_function = get_stream_functions(
        universe) if stream_cost is not None else {}

    for _ in irange(0, max_iterations):
        if verbose:
            print
        print_status(universe)
        if time() - universe.start_time >= max_time:
            break
        universe.iterations += 1
        if debug:
            focused_debug(universe)

        if check_feasible:
            plan = solve_real(universe, search, max_time)
            if plan is not None:
                assert is_real_solution(universe, plan)
                return plan, universe

        make_streams(universe, max_level)
        if debug:
            abstract_focused_debug(universe)

        atom_nodes = determine_reachable(universe)
        for instance in universe.action_instances:
            if isinstance(instance, ActionInstance):
                add_stream_cost(universe, atom_nodes, instance, stream_cost)
        plan, goals = solve_abstract(
            universe, atom_nodes, search, max_time, stream_cost)
        if verbose:
            print 'Cost:', plan_cost(universe, plan), 'Plan:', convert_plan(plan)
            raw_input('Continue?')
        if plan is None:
            if not universe.temp_blocked:
                break
            universe.temp_blocked = set()
            universe.resets += 1
            continue

        constants = set_union(set(args) for _, args in plan)
        abstract_constants = filter(
            lambda c: isinstance(c, AbstractConstant), constants)
        new_goals = goals.copy()
        new_goals.update(map(Concrete, abstract_constants))
        order = extract_streams(atom_nodes, new_goals)

        if verbose:
            print 'Goals:', len(goals)
            print 'Abstract constants:', len(abstract_constants)
            print 'Order:', order

        bound_plan = produce_bindings(
            universe, order, plan, greedy, shared, dfs)
        if verbose:
            print 'Bound plan:', bound_plan
        if bound_plan is not None:
            assert is_real_solution(universe, bound_plan)
            return bound_plan, universe
    return None, universe
