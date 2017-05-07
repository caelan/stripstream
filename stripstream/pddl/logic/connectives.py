from stripstream.pddl.logic.formulas import Formula, Condition, Effect
import stripstream.pddl.logic.atoms
from stripstream.utils import flatten
from itertools import product
import operator
from stripstream.pddl.logic import utils


def combine_connectives(formula, Class):
    if not isinstance(formula, Class):
        return [formula]
    children = []
    for f in formula.formulas:
        children += combine_connectives(f, Class)
    return children


class Connective(Formula):
    _pddl_name = None

    def __init__(self, *formulas):
        """
        Logical connective abstract class.

        :param formulas: a list of :class:`.Formula`
        """
        if not all(isinstance(f, Formula) for f in formulas):
            raise ValueError(
                'All Connective formulas must extend class Formula: %s' % formulas)
        if not formulas:
            raise ValueError('Must specify at least one Connective formula')
        self.formulas = formulas

    def get_atoms(self): return reduce(
        operator.or_, (f.get_atoms() for f in self.formulas))

    def get_formulas(self): return self.formulas

    def get_quantified(self): return reduce(
        operator.or_, (f.get_quantified() for f in self.formulas))

    def simplify(self):
        formulas = [f.simplify()
                    for f in combine_connectives(self, self.__class__)]
        if len(formulas) == 1:
            return formulas[0]
        return self.__class__(*formulas)

    def substitute(self, atom, subformula):
        new_formulas = list(self.formulas)
        for i, formula in enumerate(self.formulas):
            if isinstance(formula, stripstream.pddl.logic.atoms.Atom) and formula.predicate == atom.predicate:
                new_formulas[i] = subformula.instantiate(
                    dict(zip(atom.args, self.formulas[i].args)))
            else:
                formula.substitute(atom, subformula)
        self.formulas = tuple(new_formulas)

    def propositional(self, constants):
        return self.__class__(*[formula.propositional(constants) for formula in self.formulas])

    def dequantify(self, constants):
        return self.__class__(*[formula.dequantify(constants) for formula in self.formulas])

    def instantiate(self, parameter_map):
        return self.__class__(*[formula.instantiate(parameter_map) for formula in self.formulas])

    def pddl(self):
        return '(%s %s)' % (self._pddl_name, ' '.join(formula.pddl() for formula in self.formulas))
    __repr__ = pddl


class And(Connective, Condition, Effect):
    """
    Logical conjunction (and P1 P2 ... PN).

    .. code:: python

      BLOCK, POSE = EasyType(), EasyType()
      AtPose = EasyPredicate(BLOCK, POSE)
      CollisionFree = EasyPredicate(POSE, POSE)
      B2 = EasyParameter(BLOCK)
      P1, P2 = EasyParameter(POSE), EasyParameter(POSE)

      f = And(AtPose(B2, P2), CollisionFree(P1, P2))
    """
    _pddl_name = 'and'

    def de_morgan(self, sign=True):
        formulas = [f.de_morgan(sign=sign) for f in self.formulas]

        return And(*formulas) if sign else Or(*formulas)

    def get_literals(self):
        dnfs = [f.get_literals() for f in self.formulas]
        for combo in product(*dnfs):

            yield reduce(operator.add, combo)

    def holds(self, atoms, constants):
        return all(formula.holds(atoms, constants) for formula in self.formulas)

    def positive_supporters(self, atoms, constants):
        literals = set()
        for formula in self.formulas:
            child_literals = formula.positive_supporters(atoms, constants)
            if child_literals is None:
                return None
            literals |= child_literals
        return literals

    def negative_supporters(self, atoms, constants):
        for formula in self.formulas:
            child_literals = formula.negative_supporters(atoms, constants)
            if child_literals is not None:
                return child_literals
        return None

    def add(self, atoms, constants):
        return flatten(formula.add(atoms, constants) for formula in self.formulas)

    def delete(self, atoms, constants):
        return flatten(formula.delete(atoms, constants) for formula in self.formulas)


class Or(Connective, Condition):
    """
    Logical disjunction (or P1 P2 ... PN).

    .. code:: python

      BLOCK, POSE = EasyType(), EasyType()
      CollisionFree = EasyPredicate(BLOCK, POSE)
      B1, B2 = EasyParameter(BLOCK), EasyParameter(BLOCK)
      P1, P2 = EasyParameter(POSE)

      f = Or(Equal(B1, B2), Safe(B2, P1)))
    """
    _pddl_name = 'or'

    def de_morgan(self, sign=True):
        formulas = [f.de_morgan(sign=sign) for f in self.formulas]

        return Or(*formulas) if sign else And(*formulas)

    def get_literals(self):
        return flatten(f.get_literals() for f in self.formulas)

    def holds(self, atoms, constants):
        return any(formula.holds(atoms, constants) for formula in self.formulas)

    def positive_supporters(self, atoms, constants):
        for formula in self.formulas:
            child_literals = formula.positive_supporters(atoms, constants)
            if child_literals is not None:
                return child_literals
        return None

    def negative_supporters(self, atoms, constants):
        literals = set()
        for formula in self.formulas:
            child_literals = formula.negative_supporters(atoms, constants)
            if child_literals is None:
                return None
            literals |= child_literals
        return literals


class Not(Connective, Condition, Effect):
    """
    Logical negation (not P).

    .. code:: python

      CONF = EasyType()
      AtConf = EasyPredicate(CONF, CONF)
      Q1 = EasyParameter(CONF)

      f = Not(AtConf(Q1)))
    """
    _pddl_name = 'not'

    def __init__(self, formula):
        """
        :param formula: a :class:`.Formula`
        """
        super(Not, self).__init__(formula)

    @property
    def formula(self): return self.formulas[0]

    def de_morgan(self, sign=True): return self.formula.de_morgan(
        sign=not sign)

    def simplify(self): return self

    def get_literals(self):
        dnf = self.formulas[0].get_literals()
        for combo in product(*dnf):
            yield map(utils.invert, combo)

    def holds(self, atoms, constants):
        return not self.formula.holds(atoms, constants)

    def positive_supporters(self, atoms, constants):
        return set(map(utils.invert, self.formula.negative_supporters(atoms, constants)))

    def negative_supporters(self, atoms, constants):
        return set(map(utils.invert, self.formula.positive_supporters(atoms, constants)))

    def add(self, atoms, constants):
        return self.formula.delete(atoms, constants)

    def delete(self, atoms, constants):
        return self.formula.add(atoms, constants)

    def __eq__(self, other):
        return type(self) == type(other) and self.formula == other.formula

    def __ne__(self, other):
        return not(self == other)

    def __hash__(self):
        return hash((self.__class__, self.formulas))


class When(Connective, Effect):
    """
    Logical implication (when P1 P2).
    """
    _pddl_name = 'when'

    def __init__(self, formula1, formula2):
        """
        :param formula1: a :class:`.Formula` for the antecedent
        :param formula2: a :class:`.Formula` for the consequent
        """

        super(When, self).__init__(formula1, formula2)

    def add(self, atoms, constants):
        if self.formulas[0].holds(atoms, constants):
            return self.formulas[1].add(atoms, constants)
        return []

    def delete(self, atoms, constants):
        if self.formulas[0].holds(atoms, constants):
            return self.formulas[1].delete(atoms, constants)
        return []
