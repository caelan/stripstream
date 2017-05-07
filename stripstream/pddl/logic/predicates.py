from stripstream.pddl.objects import PARAM_PREFIX, TYPE_PAIR, Object


class Head(object):

    def __init__(self, name, types=tuple()):
        """
        Head abstract class.

        :param name: the string name of the head
        :param types: a list of :class:`.Type` inputs to the head
        .. automethod:: __call__
        """
        self.name = name.lower()
        self.types = tuple(types)
        self._hash = None

    def __call__(self, *args):
        """
        Creates an :class:`.Atom` instance from ``args``

        :param args: a list of hashable arguments to ``self``
        :returns: :class:`.Atom` with predicate ``self`` and wrapped :class:`.Object` arguments ``args``
        :raises ValueError: if ``self.types`` does not match the length of ``args``
        """
        return convert_args(self, args)

    def __eq__(self, other):
        return type(self) == type(other) and self.name == other.name and self.types == other.types

    def __ne__(self, other):
        return not(self == other)

    def __hash__(self):
        if self._hash is None:
            self._hash = hash((self.__class__, self.name, self.types))
        return self._hash

    def pddl(self):
        return '(' + ' '.join([self.name] + [TYPE_PAIR % ('%sx%s' % (PARAM_PREFIX, i), ty) for i, ty in enumerate(self.types)]) + ')'

    def __repr__(self):
        return '(' + ' '.join([self.name] + [repr(ty) for ty in self.types]) + ')'


class Predicate(Head):
    pass
    """
  Predicate P which extends :class:`.NamedHead`.
  """
NamedPredicate = Predicate


class EasyPredicate(Predicate):
    """
    Anonymous predicate P which extends :class:`.Predicate`.

    .. code:: python

      CONF = EasyType()
      AtConf = EasyPredicate(CONF)
    """
    num = 0
    template = '_pr%s'

    def __init__(self, *types):
        """
        :param types: a list of :class:`.Type` inputs to the predicate
        .. automethod:: __call__
        """
        name = self.template % EasyPredicate.num
        EasyPredicate.num += 1
        super(EasyPredicate, self).__init__(name, types)


class Function(Head):
    pass
    """
  Function F which extends :class:`.Head`.
  """

TotalCost = Function('total-cost')


class EasyFunction(Function):
    """
    Anonymous function F which extends :class:`.Function`.

    .. code:: python

      CONF = EasyType()
      Distance = EasyFunction(CONF, CONF)
    """
    num = 0
    template = '_fn%s'

    def __init__(self, *types):
        """
        :param types: a list of :class:`.Type` inputs to the predicate
        .. automethod:: __call__
        """
        name = self.template % EasyFunction.num
        EasyFunction.num += 1
        super(EasyFunction, self).__init__(name, types)


import atoms


def convert_args(self, args):
    if len(args) != len(self.types):
        raise ValueError('%s must have the same length as %s' %
                         (args, self.types))
    new_args = []
    for ty, arg in zip(self.types, args):
        if not isinstance(arg, Object):
            new_args.append(ty.get_const(arg))
        else:
            new_args.append(arg)
    return atoms.Atom(self, new_args)
