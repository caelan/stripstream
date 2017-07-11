from stripstream.fts.variable import FreeParameter, is_constant
from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.objects import OBJECT
from stripstream.pddl.cond_streams import ExternalTestCondStream

EAGER_TEST = True


class ConstraintType(object):
    num = 0

    prefix = 'Con%s'

    def __init__(self, types=(), test=None, domain=(), satisfying=(), name=None, **kwargs):
        self.types = tuple(types)
        self.test = test
        self.n = ConstraintType.num
        ConstraintType.num += 1
        self.name = name if name is not None else self.prefix % self.n
        self.domain = tuple(domain)
        self.satisfying = tuple(satisfying)
        self.predicate = Predicate(self.name, self.types)
        self.kwargs = kwargs

    def __call__(self, *args, **kwargs):
        return Constraint(self, args)

    def __repr__(self):
        return self.name

ConType = ConstraintType


Eq = ConType([OBJECT, OBJECT], test=lambda a, b: a == b, name='Eq')


Identity = ConType([OBJECT], test=lambda a: True, name='Identity')
Unconstrained = ConType([OBJECT], test=lambda a: True, name='Unconstrained')
Universe = ConType([OBJECT], test=lambda a: True, name='Universe')


DUMMY_CONSTRAINTS = (
    Identity,
    Unconstrained,
    Universe,
)


class Constraint(object):

    def __init__(self, constraint, values):
        self.constraint = constraint
        self.values = values

    def __eq__(self, other):
        return type(self) == type(other) and self.constraint == other.constraint and self.values == other.values

    def __ne__(self, other):
        return not(self == other)

    def __hash__(self):
        return hash((self.__class__, self.constraint, self.values))

    def __repr__(self):
        return '%s(%s)' % (self.constraint.name, ','.join(repr(value) for value in self.values))


def convert_constraint(constraint):

    assert all(isinstance(item, FreeParameter) for item in constraint.values)
    return constraint.constraint.predicate(*constraint.values)


def make_test_stream(constraint_form):
    assert constraint_form.test is not None
    assert not constraint_form.domain
    params = [FreeParameter('%s' % i, dtype)
              for i, dtype in enumerate(constraint_form.types)]
    constraint = constraint_form(*params)
    test = lambda inputs: constraint_form.test(
        *[i.type.get_value(i) for i in inputs])
    return ExternalTestCondStream(
        test=test,
        name=constraint_form.name,
        inputs=params,
        conditions=map(convert_constraint, constraint_form.domain),
        effects=map(convert_constraint, [constraint]),
        eager=EAGER_TEST,
        **constraint_form.kwargs
    )


def make_con_constants(con):
    return [ty(arg) if is_constant(arg) else arg
            for ty, arg in zip(con.constraint.types, con.values)]
