from math import ceil

from stripstream.algorithms.plan import substitute_axioms
from stripstream.pddl.logic.atoms import Atom, Cost, Initialize
from stripstream.pddl.logic.connectives import And
from stripstream.pddl.logic.predicates import Predicate, Function
from stripstream.pddl.objects import Constant, OBJECT, Object
from stripstream.utils import INF

Concrete = Predicate('_concrete', [OBJECT])
FUNCTION_TEMPLATE = '_%s_cost'


def is_concrete(atom):
    return atom.predicate is Concrete


def constant_from_concrete(atom):
    assert is_concrete(atom)
    [constant] = atom.args
    return constant


def partition_values(values):
    atoms, objects = set(), set()
    for atom in values:
        if isinstance(atom, Atom):
            objects.update(atom.args)
            if not is_concrete(atom):
                atoms.add(atom)
    for obj in values:
        if isinstance(obj, Object):
            objects.add(obj)
    return atoms, objects


class AbstractConstant(Constant):
    _num = 0
    _template = '_ab_%s_%s'


def make_abstract_constant(ty, shared=False):
    name = AbstractConstant._template % (ty.name, AbstractConstant._num)
    if shared:
        name = '_sh_%s_%s' % (ty.name, AbstractConstant._num)
    AbstractConstant._num += 1
    return AbstractConstant(name, ty)


def has_abstract_inputs(stream):
    return any(isinstance(inp, AbstractConstant) for inp in stream.inputs)


def replace_abstract_constants(constants, bindings):
    if any(isinstance(const, AbstractConstant) and const not in bindings for const in constants):
        return None
    return map(lambda const: bindings.get(const, const), constants)


def get_target_atoms(universe, plan, static_atoms):
    target_atoms = []
    state = universe.get_initial_atoms()
    instances = [action.instantiate(args) for action, args in plan]
    conditions = [instance.condition for instance in instances]
    for condition, instance in zip(conditions, instances) + [(universe.goal_formula, None)]:
        local_target_atoms = set()
        substitute_axioms(condition, state, universe)
        assert condition.holds(state, universe.type_to_objects)
        for condition in condition.positive_supporters(state, universe.type_to_objects):
            if condition in static_atoms:
                local_target_atoms.add(condition)
        if instance is not None:
            state = instance.apply(state, universe.type_to_objects)
        target_atoms.append(local_target_atoms)
    return target_atoms


def get_cost_atoms(action):
    return filter(lambda a: isinstance(a.predicate, Function), action.effect.get_atoms())


def get_stream_functions(universe):
    action_to_function = {}
    for action in universe.name_to_action.values():
        if get_cost_atoms(action):
            continue
        if any(atom.predicate in universe.stream_predicates for atom in action.condition.get_atoms()):

            action_to_function[action] = Function(FUNCTION_TEMPLATE % action.name, [
                                                  param.type for param in action.parameters])
            universe.add_function(action_to_function[action])
            function = action_to_function[action](*action.parameters)
            action.effect = And(action.effect, Cost(function))
            action.cost_included = True
    return action_to_function


def add_stream_cost(universe, atom_nodes, action, stream_cost):
    if action.lifted not in universe.action_to_function:
        for cost in get_cost_atoms(action):
            if any(isinstance(arg, AbstractConstant) for arg in cost.args):
                initialize = Initialize(cost, 1)

                universe.add_temp_initialize(initialize)

        return
    min_cost = INF

    for literals in [action.condition.get_atoms()]:
        cost = action.lifted.cost if action.lifted.cost is not None else 0
        for atom in literals:
            if isinstance(atom, Atom) and atom.predicate in universe.stream_predicates:
                cost += stream_cost * \
                    atom_nodes[atom][0] if atom in atom_nodes else INF
        min_cost = min(cost, min_cost)
    if min_cost < INF:
        function = universe.action_to_function[action.lifted](*action.args)
        initialize = Initialize(function, int(ceil(min_cost)))

        universe.add_temp_initialize(initialize)
