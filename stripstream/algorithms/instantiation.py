from collections import defaultdict
from itertools import product
from stripstream.pddl.logic.atoms import Initialize


def instantiate_operator(operator, objects):
    values = [objects.get(param.type, []) for param in operator.parameters]
    for combo in product(*values):
        yield operator.instantiate(combo)


def instantiate_operators(operators, objects):
    for operator in operators:
        for instance in instantiate_operator(operator, objects):
            yield instance


def instantiate_actions(universe):
    return instantiate_operators(universe.actions, universe.type_to_objects)


def instantiate_axioms(universe):
    return instantiate_operators(universe.axioms, universe.type_to_objects)


def parameter_product(conditions, condition_values, objects, object_values):
    params = {}
    combos = [[]]
    for ci, cond in enumerate(conditions):
        cond_params = dict(zip(cond.args, range(len(cond.args))))
        intersect = set(params) & set(cond_params)
        new_indices = [i for i, a in enumerate(
            cond.args) if a not in intersect]
        new_combos = []
        for combo in combos:
            for value in condition_values[ci]:
                if all(combo[params[a]] == value.args[cond_params[a]] for a in intersect):
                    new_combos.append(combo + [value.args[i]
                                               for i in new_indices])
        for i in new_indices:
            params[cond.args[i]] = len(params)
        combos = new_combos

    new_indices = [i for i, obj in enumerate(objects) if obj not in params]
    for i in new_indices:
        params[objects[i]] = len(params)
    if not params:
        return
    param_order, _ = zip(*sorted(params.items(), key=lambda (_, i): i))
    values = [object_values[i] for i in new_indices]
    for value in product(*values):
        for combo in combos:
            yield dict(zip(param_order, combo + list(value)))


def parameter_product2(parameters, parameter_values, conditions, condition_values):
    combo_index = {}
    combos = [[]]
    for cond in conditions:
        cond_index = dict(zip(cond.args, range(len(cond.args))))
        cond_parameters = set(parameters) & set(cond_index)
        intersect_parameters = cond_parameters & set(combo_index)
        free_parameters = cond_parameters - intersect_parameters
        new_indices = [i for i, a in enumerate(
            cond.args) if a in free_parameters]
        for i in new_indices:
            combo_index[cond.args[i]] = len(combo_index)
        new_combos = []
        for combo in combos:
            for value in condition_values:
                if cond.predicate == value.predicate and all(a1 in cond_parameters or a1 == a2 for a1, a2 in zip(cond.args, value.args)) and all(combo[combo_index[a]] == value.args[cond_index[a]] for a in intersect_parameters):
                    new_combos.append(combo + [value.args[i]
                                               for i in new_indices])
        combos = new_combos

    new_indices = [i for i, obj in enumerate(
        parameters) if obj not in combo_index]
    for i in new_indices:
        combo_index[parameters[i]] = len(combo_index)
    if not combo_index:
        return
    param_order, _ = zip(*sorted(combo_index.items(), key=lambda (_, i): i))
    values = [filter(lambda v: parameters[i].type == v.type,
                     parameter_values) for i in new_indices]
    for value in product(*values):
        for combo in combos:
            yield dict(zip(param_order, combo + list(value)))


def smart_instantiate_operator(operator, static_map, type_to_objects):
    arg_names = set()
    arg_values = {frozenset()}

    instances = set()

    for literals in [operator.condition.get_atoms()]:
        for literal in literals:

            if literal.predicate in static_map:
                inter_names = set(arg_names) & set(literal.args)
                inter_map = defaultdict(list)
                for static_atom in static_map[literal.predicate]:
                    if all(arg1.type == arg2.type for arg1, arg2 in zip(literal.args, static_atom.args)):
                        name_map = dict(zip(literal.args, static_atom.args))
                        inter_values = frozenset(
                            (name, name_map[name]) for name in inter_names)
                        inter_map[inter_values].append(
                            frozenset(name_map.iteritems()))

                new_arg_values = set()
                for arg_value in arg_values:
                    name_map = dict(arg_value)
                    inter_values = frozenset(
                        (name, name_map[name]) for name in inter_names)
                    for other_value in inter_map[inter_values]:
                        new_arg_values.add(arg_value | other_value)
                arg_names |= set(literal.args)
                arg_values = new_arg_values

        remaining_names = list(set(operator.parameters) - arg_names)
        remaining_values = [type_to_objects.get(
            param.type, []) for param in remaining_names]
        remaining_combos = list(product(*remaining_values))
        for arg_value in arg_values:
            for combo in remaining_combos:
                combined = dict(list(arg_value) + zip(remaining_names, combo))
                instance = operator.instantiate(
                    [combined[param] for param in operator.parameters])
                if instance not in instances:
                    instances.add(instance)
                    yield instance


def smart_instantiate_operators(operators, universe):
    static_map = {pr: [] for pr in universe.stream_predicates}
    for atom in universe.get_initial_atoms():
        if not isinstance(atom, Initialize) and atom.predicate in static_map:
            static_map[atom.predicate].append(atom)
    for operator in operators:
        for instance in smart_instantiate_operator(operator, static_map, universe.type_to_objects):
            yield instance
