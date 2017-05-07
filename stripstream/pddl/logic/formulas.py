from stripstream.utils import flatten


class Formula(object):

    def is_valid_condition(self):
        return isinstance(self, Condition) and all(f.is_valid_condition() for f in self.get_formulas())

    def is_valid_effect(self):
        return isinstance(self, Effect) and all(f.is_valid_effect() for f in self.get_formulas())

    def get_atoms(self): raise NotImplementedError()

    def get_literals(self): raise NotImplementedError()

    def get_formulas(self): raise NotImplementedError()

    def de_morgan(self, sign=True): raise NotImplementedError()

    def simplify(self): return self

    def get_objects(self):
        return set(flatten(atom.args for atom in self.get_atoms()))

    def get_parameters(self):
        return set(flatten(atom.get_parameters() for atom in self.get_atoms()))

    def get_quantified(self): raise NotImplementedError()

    def propositional(self, constants): raise NotImplementedError()

    def dequantify(self, constants): raise NotImplementedError()

    def instantiate(self, parameter_map): raise NotImplementedError()

    def clone(self): self.instantiate({})

    def substitute(self, atom, subformula): raise NotImplementedError()

    def pddl(self): raise NotImplementedError()
    __repr__ = pddl
    """
  Logical formula abstract class.
  """


class Condition():

    def holds(self, atoms, constants): raise NotImplementedError()

    def positive_supporters(
        self, atoms, constants): raise NotImplementedError()

    def negative_supporters(
        self, atoms, constants): raise NotImplementedError()
    """
  Legal condition component interface.
  """


class Effect():

    def add(self, atoms, constants): raise NotImplementedError()

    def delete(self, atoms, constants): raise NotImplementedError()
    """
  Legal effect component interface.
  """
