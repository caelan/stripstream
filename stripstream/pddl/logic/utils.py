import operator
import atoms
import connectives


def invert(literal):
    if isinstance(literal, connectives.Not):
        return literal.formulas[0]
    return connectives.Not(literal)


def is_literal(f):
    return isinstance(f, atoms.Atom) or (isinstance(f, connectives.Not) and is_literal(f.formulas[0])) or isinstance(f, atoms.Increase) or isinstance(f, atoms.Equal)


def is_conjunction(f):
    return is_literal(f) or (isinstance(f, connectives.And) and all(is_conjunction(formula)) for formula in f.formulas)


def get_literals(f):
    if is_literal(f):
        return [f]
    return reduce(operator.add, [get_literals(formula) for formula in f.formulas])


def get_increases(f):
    if isinstance(f, atoms.Increase):
        return [f]
    if isinstance(f, atoms.Atom):
        return []
    return reduce(operator.add, [get_increases(formula) for formula in f.formulas])
