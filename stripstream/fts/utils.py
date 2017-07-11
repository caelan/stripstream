from stripstream.fts.variable import VarType, FreeParameter, VariableType
from stripstream.fts.constraint import ConstraintType
from stripstream.pddl.objects import Constant
from stripstream.pddl.logic.predicates import Predicate


def get_value(x):
    if type(x) != Constant or not isinstance(x.type, VariableType):
        return x
    return x.type.get_value(x)


def convert_plan(plan):
    if plan is None:
        return plan
    return [(action, map(get_value, args)) for action, args in plan]


def rename_variables(assignments):
    for name, value in assignments.iteritems():
        if type(value) == VarType:
            value.name = name.lower()
        elif type(value) == ConstraintType:
            value.name = name.lower()
            value.predicate = Predicate(value.name, value.types)
        elif type(value) == FreeParameter:
            FreeParameter.__init__(value, name, value.type)
