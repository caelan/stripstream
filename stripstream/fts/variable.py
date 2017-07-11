from itertools import product
from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.objects import Type, Constant, Parameter as SSParameter


class VariableType(Type):
    num = 0
    prefix = '_vtype%s'

    def __init__(self, name=None, domain=None):
        if name is None:
            name = self.prefix % VariableType.num
            VariableType.num += 1
        self.value_to_const = {}
        self.const_to_value = {}
        self.domain = domain
        super(VariableType, self).__init__(name, parent=None)

    def __call__(self, value):
        if value not in self.value_to_const:
            if self.domain is not None and value not in self.domain:
                raise ValueError('%s not in the domain of %s' % (value, self))
            name = '%s_%s' % (self.name, len(self.value_to_const))
            const = Constant(name, self)
            self.value_to_const[value] = const
            self.const_to_value[const] = value
        return self.value_to_const[value]

    def get_value(self, const):
        return self.const_to_value[const]

    def param(self, name):
        return FreeParameter(name, self)

VarType = VariableType


def get_value(const):
    return const.type.get_value(const)


class FreeParameter(SSParameter):
    num = 0
    prefix = '_param%s'

    def __init__(self, name, type=None):
        if type is None:
            type = name
            assert isinstance(type, VarType)
            name = self.prefix % FreeParameter.num
            FreeParameter.num += 1
        super(FreeParameter, self).__init__(name, type)

Par = FreeParameter


def var_name(var):
    return var[0]


def var_args(var):
    return var[1:]


def split_var(var):
    return var_name(var), var_args(var)


class Variable(object):

    def __init__(self, name, dtype, args=[]):
        self.name = name
        self.dtype = dtype
        self.args = args
        assert all(arg.domain is not None for arg in args)
        self.predicate = Predicate('%s_at' % name, args + [dtype])

    def __repr__(self):
        if not self.args:
            return self.name
        return '%s(%s)' % (self.name, ','.join(map(repr, self.args)))

Var = Variable


class VariableSet(object):

    def __init__(self, name):
        self.name = name

    def __getitem__(self, var):
        if not isinstance(var, tuple):
            var = (var,)
        return VarMember(self, var)

    def __call__(self, *args):
        return self.__getitem__(args)

    def __repr__(self):
        return self.name

X = VariableSet('x')
U = VariableSet('u')
nX = VariableSet('nx')


class VariableMember(object):

    def __init__(self, temp, var):
        self.temp = temp
        self.var = var

    def __eq__(self, other):
        return type(self) == type(other) and self.temp == other.temp and self.var == other.var

    def __ne__(self, other):
        return not(self == other)

    def __hash__(self):
        return hash((self.__class__, self.temp, self.var))

    def __repr__(self):
        return '%s[%s]' % (self.temp, ','.join(repr(item) for item in self.var))

VarMember = VariableMember


def is_parameter(v):
    return isinstance(v, FreeParameter) or isinstance(v, VarMember)


def is_constant(v):
    return not is_parameter(v)


def expand_variables(vars):
    expanded_vars = set()
    for var in vars:
        values = [[var.name]] + [arg.domain for arg in var.args]
        for combo in product(*values):
            assert var not in expanded_vars
            expanded_vars.add(combo)
    return expanded_vars


def make_var_constants(var, var_map):
    name, args = split_var(var)
    assert len(args) == len(var_map[name].args)
    return [ty(arg) if not isinstance(arg, FreeParameter) else arg for ty, arg in zip(var_map[name].args, args)]
