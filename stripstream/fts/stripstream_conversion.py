from stripstream.fts.constraint import Eq, make_test_stream
from stripstream.fts.derived import get_derived
from stripstream.fts.variable import VarMember, is_constant, FreeParameter, var_args, expand_variables, split_var
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.fts.clause import convert_clause
from stripstream.fts.sampler import convert_sampler


def make_equal_predicate(item, value, var_map):
    name, args = split_var(item.var)
    if len(var_map[name].args) != len(args):
        raise RuntimeError(
            '%s must have the same number of args as %s', var_map[name].args, args)
    const_args = [ty(arg) for ty, arg in zip(var_map[name].args, args)]
    const_val = var_map[name].dtype(value)
    return var_map[name].predicate(*(const_args + [const_val]))


def get_single_state(constraints):
    state = {}
    for con in constraints:
        if not con.constraint is Eq:
            raise ValueError(
                'Initial state must solely be composed of %s constraints: %s' % (Eq, con))
        a, b = con.values
        assert isinstance(a, VarMember) and is_constant(b)
        assert not any(isinstance(arg, FreeParameter)
                       for arg in var_args(a.var))
        state[a.var] = b

    return state


def convert_initial_state(initial_state, var_map, expanded_state_vars):
    initial_atoms = []
    initial_variables = set()
    for con in initial_state:
        assert con.constraint == Eq
        a, b = con.values
        assert isinstance(a, VarMember) and is_constant(b)
        assert not any(isinstance(arg, FreeParameter)
                       for arg in var_args(a.var))
        initial_atoms.append(make_equal_predicate(a, b, var_map))
        initial_variables.add(a.var)
    if initial_variables != expanded_state_vars:
        raise ValueError(initial_variables - expanded_state_vars,
                         expanded_state_vars - initial_variables)
    return initial_atoms


def convert_goal_constraints(goal_constraints, var_map, axiom_map):

    goal_literals = []
    for con in goal_constraints:
        for item in con.values:
            assert not isinstance(item, FreeParameter) and (not isinstance(item, VarMember) or
                                                            not any(isinstance(arg, FreeParameter) for arg in item.var))
        if con.constraint == Eq:
            a, b = con.values
            if is_constant(a) and is_constant(b):
                if a != b:
                    return None
            elif isinstance(a, VarMember) and isinstance(b, VarMember):
                raise NotImplementedError()
            elif isinstance(a, VarMember) and is_constant(b):
                goal_literals.append(make_equal_predicate(a, b, var_map))
            elif is_constant(a) and isinstance(b, VarMember):
                goal_literals.append(make_equal_predicate(b, a, var_map))
            else:
                raise ValueError()
        else:

            constants = set()
            for item in con.values:
                if isinstance(item, VarMember):
                    constants.update(var_args(item.var))
                else:
                    constants.add(item)
            goal_literals.append(get_derived(
                con, var_map, axiom_map, constants))
    return goal_literals


def convert_transition(transition, var_map, axiom_map):
    actions = []
    for clause in transition:
        strips_action = convert_clause(clause, var_map, axiom_map)
        if strips_action is not None:
            actions.append(strips_action)
        else:
            print 'Warning!', clause, 'is unsatisfiable'
    return actions, axiom_map.values()


def convert_constraint_forms(con_forms, initial_atoms, cond_streams, test_streams=True):
    for con_form in con_forms:
        if test_streams and con_form.test is not None:
            cond_streams.append(make_test_stream(con_form))
        for values in con_form.satisfying:
            assert len(con_form.types) == len(values)
            objs = [ty(val) for ty, val in zip(con_form.types, values)]
            initial_atoms.append(con_form.predicate(*objs))


def convert_domains(variables):
    dtypes = set()
    for var in variables:
        dtypes.add(var.dtype)
        dtypes.update(var.args)

    objects = set()
    for dtype in dtypes:
        if dtype.domain is not None:
            for obj in dtype.domain:
                objects.add(dtype(obj))
    return objects


def constraint_to_stripstream(fts_problem, do_axioms=True, test_streams=True):
    expanded_state_vars = expand_variables(fts_problem.state_vars)

    var_map = fts_problem.get_variable_map()
    axiom_map = {}
    initial_atoms = convert_initial_state(
        fts_problem.initial_state, var_map, expanded_state_vars)
    goal_literals = convert_goal_constraints(
        fts_problem.goal_constraints, var_map, axiom_map)

    actions, axioms = convert_transition(
        fts_problem.transition, var_map, axiom_map)
    cond_streams = map(convert_sampler, fts_problem.samplers)

    convert_constraint_forms(fts_problem.get_constraint_forms(), initial_atoms, cond_streams,
                             test_streams=test_streams)
    objects = convert_domains(
        fts_problem.state_vars + fts_problem.control_vars)

    return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, objects)
