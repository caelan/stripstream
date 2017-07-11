from collections import defaultdict

from stripstream.pddl.logic.connectives import Not
from stripstream.pddl.operators import STRIPSAction
from stripstream.fts.derived import get_derived
from stripstream.fts.variable import FreeParameter, Par, X, nX, VarMember, is_parameter,  is_constant, var_args, var_name, make_var_constants
from stripstream.fts.constraint import Eq, make_con_constants, DUMMY_CONSTRAINTS, Constraint


class Clause(object):
    num = 0
    prefix = 'Clause%s'

    def __init__(self, constraints, name=None):
        for con in constraints:
            if not isinstance(con, Constraint):
                raise ValueError('%s must be a %s' %
                                 (con, Constraint.__name__))
        self.constraints = tuple(constraints)

        self.n = Clause.num
        Clause.num += 1
        self.name = name if name is not None else self.prefix % self.n

    def __repr__(self):

        return str(self.name)


def get_components(constraints):

    edges = defaultdict(set)
    for con in constraints:
        if con.constraint is Eq:
            a, b = con.values
            if a != b:
                edges[a].add(b)
                edges[b].add(a)

    components = []
    reached = set()

    def dfs(a):
        reached.add(a)
        components[-1].append(a)
        for b in edges[a]:
            if b not in reached:
                dfs(b)
    for a in list(edges):
        if a not in reached:
            components.append([])
            dfs(a)
    return components


def get_equality_map(constraints, var_map):
    equality_map = {}
    for component in get_components(constraints):
        constants = filter(is_constant, component)
        parameters = filter(is_parameter, component)

        if constants:
            assignment = constants[0]
            if any(val != assignment for val in constants):
                print 'Warning! Infeasible transition'
                return None
        else:
            assignment = component[0]
            dtypes = []
            for param in parameters:
                if isinstance(param, FreeParameter):
                    dtypes.append(param.type)
                if isinstance(param, VarMember):
                    dtypes.append(var_map[var_name(param.var)].dtype)
            if dtypes:
                dtype = dtypes[0]
                assert all(t == dtype for t in dtypes)

        for param in parameters:
            equality_map[param] = assignment
    return equality_map


def get_constraint_parameters(constraints, use_axioms):
    internal_params = set()
    for con in constraints:
        for item in con.values:
            if isinstance(item, VarMember):
                for arg in var_args(item.var):
                    if isinstance(arg, FreeParameter):
                        internal_params.add(arg)
                if item.temp != X or not use_axioms:
                    internal_params.add(item)
            elif isinstance(item, FreeParameter):
                internal_params.add(item)
    return internal_params


def get_assignments(internal_params, var_map, eq_map):
    assign_map = {}
    for param in internal_params:
        if isinstance(param, FreeParameter):
            dtype = param.type
        elif isinstance(param, VarMember):
            dtype = var_map[var_name(param.var)].dtype
        else:
            raise ValueError(param)
        eq_param = eq_map.get(param, param)
        if not is_constant(eq_param):
            if eq_param not in assign_map:
                assign_map[eq_param] = Par('%s' % len(assign_map), dtype)
            assign_map[param] = assign_map[eq_param]
        else:
            assign_map[param] = dtype(eq_param)
    return assign_map


def get_effects(var_map, effect_vars, assign_map):
    effects = []
    for var in effect_vars:
        name, args = var_name(var), make_var_constants(var, var_map)
        pre_args = [assign_map.get(p, p) for p in args + [X[var]]]
        eff_args = [assign_map.get(p, p) for p in args + [nX[var]]]
        predicate = var_map[name].predicate
        effects += [predicate(*eff_args), Not(predicate(*pre_args))]
    return effects


def get_fluent_preconditions(var_map, effect_vars, eq_map, assign_map):
    fluent_preconditions = []
    for var in effect_vars:
        name, args = var_name(var), make_var_constants(var, var_map)
        pre_args = [assign_map.get(p, p) for p in args + [X[var]]]
        predicate = var_map[name].predicate
        fluent_preconditions.append(predicate(*pre_args))
    for item in eq_map:
        if isinstance(item, VarMember) and item.temp == X and item.var not in effect_vars:
            name, args = var_name(
                item.var), make_var_constants(item.var, var_map)
            pre_args = [assign_map.get(p, p) for p in args + [X[item.var]]]
            fluent_preconditions.append(var_map[name].predicate(*pre_args))
    return fluent_preconditions


def get_static_preconditions(constraints, var_map, internal_params, assign_map, axiom_map):

    static_preconditions = []
    for con in constraints:
        if con.constraint != Eq and con.constraint not in DUMMY_CONSTRAINTS:
            if all(not isinstance(item, VarMember) or item in internal_params for item in con.values):
                values = make_con_constants(con)
                static_preconditions.append(con.constraint.predicate(
                    *[assign_map.get(item, item) for item in values]))
            else:

                constants = set()
                new_values = []
                for item in con.values:
                    if not isinstance(item, VarMember) or item in internal_params:
                        new_values.append(assign_map.get(item, item))
                        constants.add(assign_map.get(item, item))
                    else:
                        constants.update(assign_map.get(arg, arg)
                                         for arg in var_args(item.var))
                        new_values.append(
                            X(*[assign_map.get(arg, arg) for arg in item.var]))
                derived = get_derived(con.constraint(
                    *new_values), var_map, axiom_map, constants)
                if derived not in static_preconditions:
                    static_preconditions.append(derived)
    return static_preconditions


def convert_clause(clause, var_map, axiom_map):
    effect_vars = {item.var for con in clause.constraints for item in con.values
                   if isinstance(item, VarMember) and item.temp == nX}
    eq_map = get_equality_map(clause.constraints, var_map)
    if eq_map is None:
        return None

    internal_params = set(eq_map) | {X[var] for var in effect_vars} | {
        nX[var] for var in effect_vars} | get_constraint_parameters(clause.constraints, axiom_map is not None)

    assign_map = get_assignments(internal_params, var_map, eq_map)

    clause.parameter_map = {
        v: p for v, p in assign_map.iteritems() if isinstance(p, FreeParameter)}

    parameters = filter(lambda p: isinstance(
        p, FreeParameter), assign_map.values())
    preconditions = get_fluent_preconditions(var_map, effect_vars, eq_map, assign_map) + get_static_preconditions(
        clause.constraints, var_map, internal_params, assign_map, axiom_map)
    effects = get_effects(var_map, effect_vars, assign_map)

    action = STRIPSAction(clause.name, parameters, preconditions, effects)
    action.clause = clause
    return action
