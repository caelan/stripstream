from itertools import product

import stripstream.pddl.logic.atoms
from stripstream.pddl.logic.connectives import Connective, And, Or
from stripstream.pddl.logic.formulas import Formula, Condition, Effect
from stripstream.pddl.objects import Parameter


def combine_quantifiers(formula, Class):
    if not isinstance(formula, Class):
        return [], formula
    args, base_formula = combine_quantifiers(formula.formula, Class)
    return formula.args + args, base_formula


class Quantifier(Formula):
    _pddl_name = None
    _Connective = None

    def __init__(self, args, formula):
        """
        Quantifier abstract class.

        :param args: a list of :class:`.Parameter` free variables
        :param formula: the :class:`.Formula` within the quantifier
        """
        if not all(isinstance(arg, Parameter) for arg in args):
            raise ValueError(
                'All quantifier arguments must extend class Parameter: %s' % args)
        if not isinstance(formula, Formula):
            raise ValueError(
                'Quantifier formula must extend class Formula: %s' % formula)
        self.args = args
        self.formula = formula

    def get_atoms(self): return self.formula.get_atoms()

    def get_formulas(self): return [self.formula]

    def get_quantified(self):
        return set(self.args) | self.formula.get_quantified()

    def simplify(self):
        args, formula = combine_quantifiers(self, self.__class__)
        if not args:
            return formula.simplify()
        return self.__class__(args, formula.simplify())

    def substitute(self, atom, subformula):
        if isinstance(self.formula, stripstream.pddl.logic.atoms.Atom) and self.formula.predicate == atom.predicate:
            self.formula = subformula.instantiate(
                dict(zip(atom.args, self.formula.args)))
        else:
            self.formula.substitute(atom, subformula)

    def instantiate(self, parameter_map):
        return self.__class__(self.args, self.formula.instantiate(parameter_map))

    def expand(self, constants):
        values = [constants.get(arg.type, []) for arg in self.args]
        for combo in product(*values):
            yield self.formula.instantiate(dict(zip(self.args, combo)))

    def propositional(self, constants):
        formulas = [f.propositional(constants) for f in self.expand(constants)]
        return self._Connective(*formulas)

    def pddl(self):
        return '(%s (%s) %s)' % (self._pddl_name, ' '.join(arg.typed_pddl() for arg in self.args), self.formula.pddl())
    __repr__ = pddl


def has_existential_child(formula):
    if isinstance(formula, Exists):
        return True
    if isinstance(formula, Connective):
        return any(has_existential_child(f) for f in formula.formulas)
    if isinstance(formula, Quantifier):
        return has_existential_child(formula.formula)
    return False


class ForAll(Quantifier, Condition, Effect):
    """
    Universal quantifier (forall (X1 X2 ... XN) P).

    Example safe for all blocks formula:

    .. code:: python

      BLOCK, POSE = EasyType(), EasyType()
      Safe = EasyPredicate(BLOCK, POSE)
      B1, B2 = EasyParameter(BLOCK), EasyParameter(BLOCK)
      P1 = EasyParameter(POSE)

      f = ForAll([B2], Or(Equal(B1, B2), Safe(B2, P1)))
    """
    _pddl_name = 'forall'
    _Connective = And

    def de_morgan(self, sign=True):
        formula = self.formula.de_morgan(sign=sign)
        return ForAll(self.args, formula) if sign else Exists(self.args, formula)

    def dequantify(self, constants):
        if has_existential_child(self.formula):
            raise NotImplementedError(
                'Currently do not support ForAll([...], Exists(...))')
        print 'Warning: ForAll propositionalized using a fixed set of constants'
        formulas = [f.dequantify(constants) for f in self.expand(constants)]
        return self._Connective(*formulas)

    def holds(self, atoms, constants):
        return all(formula.holds(atoms, constants) for formula in self.expand(constants))

    def positive_supporters(self, atoms, constants):
        literals = set()
        for formula in self.expand(constants):
            child_literals = formula.positive_supporters(atoms, constants)
            if child_literals is None:
                return None
            literals |= child_literals
        return literals

    def negative_supporters(self, atoms, constants):
        for formula in self.expand(constants):
            child_literals = formula.negative_supporters(atoms, constants)
            if child_literals is not None:
                return child_literals
        return None

    def add(self, atoms, constants):
        return [formula.add(atoms, constants) for formula in self.expand(constants)]

    def delete(self, atoms, constants):
        return [formula.delete(atoms, constants) for formula in self.expand(constants)]


class Exists(Quantifier, Condition):
    """
    Existential quantifier (exists (X1 X2 ... XN) P).

    Example collision free for some pose formula:

    .. code:: python

      BLOCK, POSE = EasyType(), EasyType()
      AtPose = EasyPredicate(BLOCK, POSE)
      CollisionFree = EasyPredicate(POSE, POSE)
      B2 = EasyParameter(BLOCK)
      P1, P2 = EasyParameter(POSE), EasyParameter(POSE)

      f = Exists([P2], And(AtPose(B2, P2), CollisionFree(P1, P2)))
    """
    _pddl_name = 'exists'
    _Connective = Or

    def de_morgan(self, sign=True):
        formula = self.formula.de_morgan(sign=sign)
        return Exists(self.args, formula) if sign else ForAll(self.args, formula)

    def dequantify(self, constants):
        return self.formula.dequantify(constants)

    def holds(self, atoms, constants):
        return any(formula.holds(atoms, constants) for formula in self.expand(constants))

    def positive_supporters(self, atoms, constants):
        for formula in self.expand(constants):
            child_literals = formula.positive_supporters(atoms, constants)
            if child_literals is not None:
                return child_literals
        return None

    def negative_supporters(self, atoms, constants):
        literals = set()
        for formula in self.expand(constants):
            child_literals = formula.negative_supporters(atoms, constants)
            if child_literals is None:
                return None
            literals |= child_literals
        return literals
