from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.objects import Parameter
from stripstream.pddl.streams import Stream, TestStream, GeneratorStream, FunctionStream
from stripstream.pddl.logic.atoms import Initialize
from stripstream.pddl.logic.predicates import Function
from stripstream.pddl.objects import OBJECT,  Parameter
from stripstream.pddl.logic.predicates import NamedPredicate
import collections
import math
import numbers

DEFAULT_COST = 1
TEST_COST = 0
PLAN_TIME = 1.


def log_cost(p, min_p=1e-6):
    return -math.log(max(p, min_p))


def geom_cost(cost, p, min_p=1e-6):
    return cost / max(p, min_p)


def mdp_cost(success_cost, failure_cost, p, min_p=1e-6):

    return success_cost + failure_cost * (1. / max(p, min_p) - 1.)


class ConditionalStream(object):
    _num = 0

    def __init__(self, inputs, outputs, conditions, effects,
                 eager=False, cost=DEFAULT_COST, order=0, plannable=True,
                 prob=1., avg_time=1., max_level=None):
        """
        Conditional stream abstract class.

        :param inputs: a list of :class:`.Parameter` which are the inputs to ``StreamFn``
        :param outputs: a list of :class:`.Parameter` which are the outputs from ``StreamFn``
        :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
        :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs
        :param eager: boolean which informs a planner to apply ``self`` immediately
        :param cost: numeric which informs a planner of the difficultly (ex. runtime, likelihood of success) in applying ``self``
        :param order: numeric which informs a planner of the order to apply ``self``
        :param plannable: boolean which informs a planner if ``self`` can be treated as an action
        """
        if not all(isinstance(p, Parameter) for p in inputs):
            raise ValueError('All inputs must be class Parameter: %s' % inputs)
        if not all(isinstance(p, Parameter) for p in outputs):
            raise ValueError(
                'All outputs must be of class Parameter: %s' % outputs)
        if set(inputs) & set(outputs):
            raise ValueError('The input and output parameters must be distinct: %s' % set(
                inputs) & set(outputs))
        if not all(isinstance(atom, Atom) for atom in conditions):
            raise ValueError(
                'All conditions must be of class Atom: %s' % conditions)
        if not all(isinstance(atom, Atom) for atom in effects):
            raise ValueError('All effects must be of class Atom: %s' % effects)
        con_params = {
            p for con in conditions for p in con.args if isinstance(p, Parameter)}
        if not (con_params <= set(inputs)):
            raise ValueError('Parameters %s within the conditions must be specified within the inputs %s' % (
                con_params, set(inputs)))
        eff_params = {
            p for eff in effects for p in eff.args if isinstance(p, Parameter)}
        if not (eff_params <= set(list(inputs) + list(outputs))):
            raise ValueError('Parameters %s within the effects must be specified within the inputs %s or outputs %s' % (
                eff_params, set(inputs), set(outputs)))

        self.name = self.__class__.__name__
        self.inputs = tuple(inputs)
        self.outputs = tuple(outputs)
        self.conditions = conditions
        self.effects = effects
        self.free_params = tuple(set(inputs) - con_params)
        self.free_outputs = tuple(set(outputs) - eff_params)
        self.streams = {}
        self.eager = eager
        self.cost = cost
        self.order = order
        self.plannable = plannable
        self.prob = prob
        self.avg_time = avg_time
        self.mdp_cost = mdp_cost(
            self.avg_time, self.avg_time + PLAN_TIME, self.prob)
        self.max_level = max_level
        self.n = CondStream._num
        CondStream._num += 1

    StreamFn = Stream

    def instantiate_conditions(self, inputs):
        param_map = dict(zip(self.inputs, inputs))
        return [condition.instantiate(param_map) for condition in self.conditions]

    def instantiate_effects(self, inputs, outputs):
        param_map = dict(zip(self.inputs, inputs) + zip(self.outputs, outputs))
        return [effect.instantiate(param_map) for effect in self.effects]

    def __call__(self, inputs):
        inputs = tuple(inputs)
        if inputs not in self.streams:
            self.streams[inputs] = self.StreamFn(self, inputs)
        return self.streams[inputs]

    def reset(self):
        for stream in self.streams.values():
            stream.reset()

    def all_values(self):
        for stream in self.streams.values():
            for values in stream.call_history():
                for value in values:
                    yield value

    @property
    def calls(self):
        return sum(stream.calls for stream in self.streams.values())

    @property
    def call_time(self):
        return sum(stream.call_time for stream in self.streams.values())

    def __repr__(self):

        return '%s->%s' % (self.inputs, self.outputs)

CondStream = ConditionalStream


class ConstCondStream(CondStream):

    def __init__(self, outputs, effects, **kwargs):
        super(ConstCondStream, self).__init__(
            [], outputs, [], effects, **kwargs)


class TestCondStream(CondStream):

    def __init__(self, inputs, conditions, effects, **kwargs):
        super(TestCondStream, self).__init__(inputs, [],
                                             conditions, effects, cost=TEST_COST, **kwargs)


class ProducerCondStream(CondStream):

    def __init__(self, inputs, outputs, **kwargs):
        super(ProducerCondStream, self).__init__(
            inputs, outputs, [], [], **kwargs)


class CostCondStream(CondStream):

    def __init__(self, inputs, conditions, function, **kwargs):
        super(CostCondStream, self).__init__(inputs, [],
                                             conditions, [function], cost=TEST_COST, **kwargs)


class ExternalTestCondStream(TestCondStream):

    def __init__(self, test, name=None, *args, **kwargs):
        self.test = test
        super(ExternalTestCondStream, self).__init__(*args, **kwargs)
        if name is not None:
            self.name = name

    class StreamFn(TestStream):

        def __init__(self, cs, inputs, **kwargs):
            self.test = cs.test
            TestStream.__init__(self, cs, inputs, **kwargs)


class ExternalGenCondStream(CondStream):

    def __init__(self, generator, name=None, *args, **kwargs):
        self.generator = generator
        super(ExternalGenCondStream, self).__init__(*args, **kwargs)
        if name is not None:
            self.name = name

    class StreamFn(GeneratorStream):

        def __init__(self, cs, inputs, **kwargs):
            self.get_generator = cs.generator
            GeneratorStream.__init__(self, cs, inputs, **kwargs)


class EasyGenStream(CondStream):
    """
    Conditional stream given by a generator.

    Example for inverse kinematics:

    .. code:: python

      CONF, POSE = EasyType(), EasyType()
      LegalKin = EasyPredicate(POSE, CONF)
      Q, P = EasyParameter(CONF), EasyParameter(POSE)

      cs = EasyGenStream(inputs=[P], outputs=[Q], conditions=[], effects=[LegalKin(P, Q)],
                         generator=lambda p: iter([p]))
    """

    def __init__(self, inputs, outputs, conditions, effects, generator, **kwargs):
        """
        :param inputs: a list of :class:`.Parameter` which are the inputs to ``generator``
        :param outputs: a list of :class:`.Parameter` which are the outputs from ``generator``
        :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
        :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs and outputs
        :param generator: a function from values for ``inputs`` to a generator for values of ``outputs``
        :param kwargs: keyword arguments for :class:`.ConditionalStream`
        """
        super(EasyGenStream, self).__init__(
            inputs, outputs, conditions, effects, **kwargs)

        if not callable(generator):
            raise ValueError(
                'EasyGenStream expects generator to be a function: %s' % generator)
        self.generator = generator

    class StreamFn(GeneratorStream):

        def get_generator(self, inputs):
            sequence = self.cond_stream.generator(
                *tuple(inp.type.get_value(inp) for inp in inputs))

            for i, outputs in enumerate(sequence):

                if len(outputs) != len(self.cond_stream.outputs):
                    raise ValueError('Expected generator output %s to be the same length as conditional stream inputs %s' % (
                        outputs, self.cond_stream.outputs))
                yield [tuple(par.type(out) for par, out in zip(self.cond_stream.outputs, outputs))]


class MultiEasyGenStream(EasyGenStream):

    class StreamFn(GeneratorStream):

        def get_generator(self, inputs):
            values = tuple(inp.type.get_value(inp) for inp in inputs)

            for output_list in self.cond_stream.generator(*values):
                assert type(output_list) == list
                new_output_list = []
                for outputs in output_list:
                    if type(outputs) not in [list, tuple]:
                        outputs = [outputs]
                    assert len(outputs) == len(self.cond_stream.outputs)
                    new_output_list.append(
                        tuple(par.type(out) for par, out in zip(self.cond_stream.outputs, outputs)))

                yield new_output_list


class EasyTestStream(CondStream):
    """
    Conditional stream given by a test.

    Example for collision checking:

    .. code:: python

      POSE = EasyType()
      CollisionFree = EasyPredicate(POSE, POSE)
      P1, P1 = EasyParameter(POSE), EasyParameter(POSE)

      cs = EasyTestStream(inputs=[P1, P2], conditions=[], effects=[CollisionFree(P1, P2)],
                          test=lambda p1, p2: p1 != p2)
    """

    def __init__(self, inputs, conditions, effects, test, **kwargs):
        """
        :param inputs: a list of :class:`.Parameter` which are the inputs to ``test``
        :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
        :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs
        :param test: a function from values for ``inputs`` to ``{False, True}``
        :param kwargs: keyword arguments for :class:`.ConditionalStream`
        """
        super(EasyTestStream, self).__init__(inputs, [],
                                             conditions, effects, cost=TEST_COST, **kwargs)
        if not callable(test):
            raise ValueError(
                'EasyTestStream expects test to be a function: %s' % test)
        self.test = test

    class StreamFn(TestStream):

        def test(self, inputs):
            values = tuple(inp.type.get_value(inp) for inp in inputs)
            truth = self.cond_stream.test(*values)
            if truth not in (True, False):
                raise ValueError(
                    'Expected boolean test output but received %s' % truth)
            return truth


class EasyFnStream(CondStream):
    """
    Conditional stream given by a function.
    """

    def __init__(self, inputs, outputs, conditions, effects, function, test=lambda *args: True, **kwargs):
        """
        :param inputs: a list of :class:`.Parameter` which are the inputs to ``generator``
        :param outputs: a list of :class:`.Parameter` which are the outputs from ``generator``
        :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
        :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs and outputs
        :param function: a function from values for ``inputs`` to values of ``outputs``
        :param test: an optional function from values for ``inputs`` to ``{False, True}`` # TODO
        :param kwargs: keyword arguments for :class:`.ConditionalStream`
        """
        super(EasyFnStream, self).__init__(
            inputs, outputs, conditions, effects, **kwargs)
        if not callable(function):
            raise ValueError(
                'EasyFnStream expects function to be a function: %s' % function)
        self.function = function
        self.test = test

    class StreamFn(FunctionStream):

        def test(self, inputs):
            in_values = tuple(inp.type.get_value(inp) for inp in inputs)
            return self.cond_stream.test(*in_values)

        def function(self, inputs):
            in_values = tuple(inp.type.get_value(inp) for inp in inputs)
            out_values = self.cond_stream.function(*in_values)
            if type(out_values) != list:
                out_values = [out_values]
            if len(out_values) != len(self.cond_stream.outputs):
                raise ValueError('Expected generator output %s to be the same length as conditional stream inputs %s' % (
                    out_values, self.cond_stream.outputs))
            return tuple(par.type(out) for par, out in zip(self.cond_stream.outputs, out_values))


class ClassStream(CondStream):

    def __init__(self, inputs, outputs, conditions, effects, StreamClass, **kwargs):
        super(ClassStream, self).__init__(
            inputs, outputs, conditions, effects, **kwargs)
        if not issubclass(StreamClass, Stream):
            raise ValueError(
                'ClassStream expects StreamClass to extend %s: %s' % (Stream, StreamClass))
        self.StreamFn = StreamClass


class EasyCostStream(CondStream):

    def __init__(self, inputs, conditions, effects, function, scale=100, eager=True, **kwargs):
        """
        :param inputs: a list of :class:`.Parameter` which are the inputs to ``test``
        :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
        :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs
        :param function: a function from values for ``inputs`` to a nonnegative number
        :param kwargs: keyword arguments for :class:`.ConditionalStream`
        """
        super(EasyCostStream, self).__init__(inputs, [],
                                             conditions, effects, eager=eager, **kwargs)
        if not callable(function):
            raise ValueError(
                'EasyCostStream expects function to be a function: %s' % function)
        assert all(isinstance(effect.predicate, Function)
                   for effect in effects)
        self.function = function
        self.scale = scale

    class StreamFn(Stream):

        def get_values(self, **kwargs):
            self.enumerated = True
            values = tuple(inp.type.get_value(inp) for inp in self.inputs)
            cost = self.cond_stream.function(*values)
            assert isinstance(cost, numbers.Number) and cost >= 0
            cost = int(self.cond_stream.scale * cost)
            return map(lambda f: Initialize(f, cost), self.cond_stream.instantiate_effects(self.inputs, []))


def equal_stream(ty1=OBJECT, ty2=OBJECT):
    name = '_eq_%s_%s' % (ty1.name, ty2.name)
    AreEqual = NamedPredicate(name, [ty1, ty2])
    X1, X2 = Parameter('x1', ty1), Parameter('x2', ty2)
    cs = EasyTestStream([X1, X2], [], [AreEqual(X1, X2)],
                        lambda x1, x2: x1 == x2, eager=True)
    return AreEqual, cs


def not_equal_stream(ty1=OBJECT, ty2=OBJECT):
    name = '_not_eq_%s_%s' % (ty1.name, ty2.name)
    AreNotEqual = NamedPredicate(name, [ty1, ty2])
    X1, X2 = Parameter('x1', ty1), Parameter('x2', ty2)
    cs = EasyTestStream([X1, X2], [], [AreNotEqual(X1, X2)],
                        lambda x1, x2: x1 != x2, eager=True)
    return AreNotEqual, cs


def make_test_stream(cond_stream):
    if not cond_stream.outputs:
        return cond_stream

    raise NotImplementedError()
