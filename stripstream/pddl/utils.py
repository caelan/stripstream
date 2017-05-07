from stripstream.pddl.logic.connectives import Not
from stripstream.pddl.logic.utils import is_literal
from stripstream.pddl.objects import Parameter, Constant
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.objects import EasyType, EasyParameter


def get_value(x):
    if type(x) != Constant or not isinstance(x.type, EasyType):
        return x
    return x.type.get_value(x)


def convert_plan(plan):
    if plan is None:
        return plan
    return [(action, map(get_value, args)) for action, args in plan]


def get_param_names(literals):
    param_names = {}
    for literal in literals:
        assert is_literal(literal)
        atom = literal.formula if isinstance(literal, Not) else literal
        for arg in atom.args:
            if isinstance(arg, Parameter):
                if arg.name in param_names:
                    assert param_names[arg.name] == arg
                else:
                    param_names[arg.name] = arg
    return param_names


def rename_easy(assignments):
    """
    Mutates any :class:`.EasyType`, :class:`.EasyPredicate`, or :class:`.EasyParameter` instance to assign their local scope name

    :param assignments: the result of ```locals()```
    """
    for name, value in assignments.iteritems():
        if type(value) in [EasyType, EasyPredicate]:
            value.name = name.lower()
        elif type(value) == EasyParameter:
            Parameter.__init__(value, name, value.type)


def reset_easy():
    """
    Mutates the :class:`.EasyType`, :class:`.EasyPredicate`, and :class:`.EasyParameter` classes themselves to reset ```_num```
    """
    for Class in (EasyType, EasyPredicate, EasyParameter):
        Class._num = 0
