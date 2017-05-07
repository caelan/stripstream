from stripstream.pddl.logic.utils import is_literal
from stripstream.pddl.logic.atoms import Atom, Cost
from stripstream.pddl.logic.quantifiers import Exists
from stripstream.pddl.logic.connectives import And, Not
from stripstream.pddl.objects import Constant, Parameter
from stripstream.pddl.utils import get_param_names
from stripstream.utils import flatten


class Operator(object):

    def __init__(self, parameters, condition, effect):
        """
        PDDL Operator abstract class.

        :param parameters: a list of :class:`.Parameter`
        :param condition: a :class:`.Condition`
        :param effect: a :class:`.Effect`
        """
        condition, effect = condition.simplify().de_morgan(), effect.simplify().de_morgan()
        if not all(isinstance(p, Parameter) for p in parameters):
            raise ValueError(
                'Parameters must all extend class Parameter: %s' % parameters)
        if not condition.is_valid_condition():
            raise ValueError('%s is not a valid condition' % condition)
        if not effect.is_valid_effect():
            raise ValueError('%s is not a valid effect' % effect)
        c_params = condition.get_parameters()
        if not (c_params <= set(parameters) | condition.get_quantified()):
            raise ValueError(
                'Condition uses unbound parameters: %s' % c_params)
        e_params = effect.get_parameters()
        if not (e_params <= set(parameters) | effect.get_quantified()):
            raise ValueError('Effect uses unbound parameters: %s' % e_params)
        if not (set(parameters) <= c_params | e_params):
            print 'Warning: unused parameters %s' % (c_params | e_params - set(parameters))
        self.parameters = tuple(parameters)
        self.condition = condition
        self.effect = effect

    def get_parameters(self):
        return set(self.condition.get_parameters()) | set(self.effect.get_parameters())

    def add_conditions(self, *new_conditions):
        self.condition = And(self.condition, *new_conditions)

    def add_effects(self, *new_effects):
        self.effect = And(self.effect, *new_effects)


class STRIPS:

    def add_conditions(self, *new_conditions):
        self.conditions += new_conditions

        super(self.__class__, self).add_conditions(*new_conditions)

    def pos_conditions(self):
        return filter(lambda a: isinstance(a, Atom), self.conditions)

    def neg_conditions(self):
        return [a.formula for a in self.conditions if isinstance(a, Not)]


class Action(Operator):
    """
    PDDL action class.

    .. code:: python

      CONF = EasyType()
      AtConf = EasyPredicate(CONF, CONF)
      Q1, Q1 = EasyParameter(CONF), EasyParameter(CONF)

      a = Action(name='move', parameters=[Q1, Q2],
                 condition=AtConf(Q1),
                 effect=And(AtConf(Q2), Not(AtConf(Q1))))
    """

    def __init__(self, name, parameters, condition, effect, cost=None):
        """
        :param name: the name of the action
        :param parameters: a list of :class:`.Parameter`
        :param condition: a :class:`.Condition`
        :param effect: an :class:`.Effect`
        :param cost: a numeric cost
        """
        super(Action, self).__init__(parameters, condition, effect)
        self.name = name.lower()
        self.cost_included = False
        self.cost = cost

    def instantiate(self, args): return ActionInstance(self, args)

    def pddl(self, costs):
        s = '(:action ' + self.name + '\n'
        s += '\t:parameters (' + ' '.join(param.typed_pddl()
                                          for param in self.parameters) + ')\n'
        s += '\t:precondition ' + self.condition.pddl() + '\n'
        augmented_effect = self.effect
        if costs and self.cost is not None and not self.cost_included:
            augmented_effect = And(augmented_effect, Cost(self.cost))
        s += '\t:effect ' + augmented_effect.pddl()
        return s + ')'

    def to_strips(self, constants):
        condition = self.condition.dequantify(constants)
        effect = self.effect.dequantify(constants)
        all_parameters = condition.get_parameters() | effect.get_parameters()
        parameters = list(self.parameters) + \
            [p for p in all_parameters if p not in self.parameters]
        [effs] = effect.get_literals()
        for i, cons in enumerate(condition.get_literals()):
            instance = STRIPSAction('%s_%s' % (
                self.name, i), parameters, cons, effs)
            instance.original = self
            yield instance

    def __repr__(self):
        return self.name
    __str__ = __repr__


class STRIPSAction(Action, STRIPS):
    """
    STRIPS action class which extends :class:`.Action` by assuming
    ``condition`` and ``effect`` are conjunctions of literals.

    .. code:: python

      CONF = EasyType()
      AtConf = EasyPredicate(CONF, CONF)
      Q1, Q1 = EasyParameter(CONF), EasyParameter(CONF)

      a = Action(name='move', parameters=[Q1, Q2],
                 conditions=[AtConf(Q1)],
                 effects=[AtConf(Q2), Not(AtConf(Q1))])
    """

    def __init__(self, name, parameters, conditions, effects, cost=None):
        """
        :param name: the string name of the action
        :param parameters: a list of :class:`.Parameter`
        :param conditions: a list of :class:`.Atom` or :class:`.Not`
        :param effects: a list of :class:`.Atom` or :class:`.Not`
        :param cost: a numeric cost
        """
        if not all(is_literal(c) for c in conditions):
            raise ValueError(
                'Conditions must all be literals: %s' % conditions)
        if not all(is_literal(e) for e in effects):
            raise ValueError('Effects must all be literals: %s' % effects)
        self.conditions, self.effects = conditions, effects
        super(STRIPSAction, self).__init__(name, parameters,
                                           And(*conditions), And(*effects), cost=cost)

    def add_effects(self, *new_effects):
        self.effects += new_effects

        super(STRIPSAction, self).add_effects(*new_effects)

    def add_list(self):
        return filter(lambda a: isinstance(a, Atom), self.effects)

    def delete_list(self):
        return [a.formula for a in self.effects if isinstance(a, Not)]


class Axiom(Operator):
    """
    PDDL axiom (derived predicate).

    .. code:: python

      BLOCK, POSE = EasyType(), EasyType()
      AtPose = EasyPredicate(BLOCK, POSE)
      CollisionFree = EasyPredicate(POSE, POSE)
      Safe = EasyPredicate(BLOCK, POSE)
      B2 = EasyParameter(BLOCK)
      P1, P2 = EasyParameter(POSE), EasyParameter(POSE)

      x = Axiom(effect=Safe(B2, P1),
                condition=Exists([P2], And(AtPose(B2, P2), CollisionFree(P1, P2))))
    """

    def __init__(self, effect, condition):
        """
        :param effect: the :class:`.Atom` derived predicate
        :param condition: a :class:`.Condition`
        """
        if not isinstance(effect, Atom):
            raw_input('Axiom effect must be an Atom: %s' % effect)

        super(Axiom, self).__init__(effect.args, condition, effect)

    def instantiate(self, args): return AxiomInstance(self, args)

    def to_strips(self, constants): raise NotImplementedError()

    def pddl(self):
        return '(:derived %s\n\t%s)' % (self.effect.typed_pddl(), self.condition.pddl())

    def __repr__(self):
        return self.effect.name
    __str__ = __repr__


class STRIPSAxiom(Axiom, STRIPS):
    """
    STRIPS axiom class which extends :class:`.Axiom` by assuming
    ``condition`` is a conjunction of literals.

    .. code:: python

      BLOCK, POSE = EasyType(), EasyType()
      AtPose = EasyPredicate(BLOCK, POSE)
      CollisionFree = EasyPredicate(POSE, POSE)
      Safe = EasyPredicate(BLOCK, POSE)
      B2 = EasyParameter(BLOCK)
      P1, P2 = EasyParameter(POSE), EasyParameter(POSE)

      x = Axiom(conditions=[AtPose(B2, P2), CollisionFree(P1, P2)],
                effects=[Safe(B2, P1)])
    """

    def __init__(self, conditions, effects):
        """
        :param conditions: a list of :class:`.Atom` or :class:`.Not`
        :param effects: a list of :class:`.Atom` or :class:`.Not`
        """
        if len(effects) != 1:
            raise NotImplementedError(
                'Currently only support a single effect: %s' % effects)
        if not all(is_literal(c) for c in conditions):
            raise ValueError(
                'Conditions must all be literals: %s' % conditions)
        if not all(is_literal(e) for e in effects):
            raise ValueError('Effects must all be literals: %s' % effects)
        self.conditions, self.effects = conditions, effects
        self.free_parameters = list(set(flatten(
            c.get_parameters() for c in conditions)) - set(effects[0].get_parameters()))
        super(STRIPSAxiom, self).__init__(
            effects[0], Exists(self.free_parameters, And(*conditions)))


class OperatorInstance(object):

    def __init__(self, lifted, args):
        self.lifted = lifted
        self.args = tuple(args)
        param_map = dict(zip(self.lifted.parameters, args))
        self.condition = lifted.condition.instantiate(param_map)
        self.effect = lifted.effect.instantiate(param_map)
        self._hash = None

    def is_applicable(self, atoms, constants):
        return self.condition.holds(atoms, constants)

    def apply(self, atoms, constants):
        add, delete = set(self.effect.add(atoms, constants)), set(
            self.effect.delete(atoms, constants))
        return (atoms | (add - delete)) - (delete - add)

    def __eq__(self, other):
        return type(self) == type(other) and self.lifted == other.lifted and self.args == other.args

    def __ne__(self, other): return not self == other

    def __hash__(self):
        if not self._hash:
            self._hash = hash((self.__class__, self.lifted, self.args))
        return self._hash


class ActionInstance(OperatorInstance):

    def __init__(self, lifted, args):
        self.cost = lifted.cost if lifted.cost is not None else 0
        super(ActionInstance, self).__init__(lifted, args)

    @property
    def name(self):
        return self.lifted.name + '(' + ','.join(arg.name for arg in self.args) + ')'

    def __repr__(self): return self.name
    __str__ = __repr__


class AxiomInstance(OperatorInstance):

    def __repr__(self): return repr(self.effect)
    __str__ = __repr__
