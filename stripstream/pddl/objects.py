import collections

PARAM_PREFIX = '?'
TYPE_PAIR = '%s - %s'


class Type(object):

    def __init__(self, name, parent=None):
        """
        PDDL type.

        :param name: the string name of the type
        :param parent: the parent :class:`.Type`
        """
        self.name = name.lower()
        self.parent = parent
        self._hash = None

    def __eq__(self, other):
        return type(self) == type(other) and self.name == other.name

    def __ne__(self, other):
        return not(self == other)

    def __hash__(self):
        if self._hash is None:
            self._hash = hash((self.__class__, self.name))
        return self._hash

    def __repr__(self):
        return self.name

    def pddl(self):
        return self.name

OBJECT = Type('object')


class FixedType(Type):

    def __init__(self, name, universe, **kwargs):
        self.universe = universe
        super(FixedType, self).__init__(name, **kwargs)


class FiniteType(Type):
    pass


class DiscreteType(Type):
    pass


class UnknownType(Type):
    pass


class EuclideanType(Type):

    def __init__(self, name, norm):
        self.norm = norm
        super(EuclideanType, self).__init__(name)


class EasyType(Type):
    """
    Anonymous type which extends :class:`.Type` by automatically generating a unique name.

    .. code:: python

      CONF = EasyType()
    """
    _num = 0
    _template = '_ty%s'

    _const_template = '%s_%s'

    def __init__(self):
        name = self._template % EasyType._num
        EasyType._num += 1
        self.const_to_value = {}
        self.value_to_const = {}
        super(EasyType, self).__init__(name)

    def get_const(self, value):
        if not isinstance(value, collections.Hashable):
            raise ValueError('%s is not hashable' % value)
        if value not in self.value_to_const:

            name = self._const_template % (self.name, len(self.value_to_const))
            self.value_to_const[value] = Constant(name, self)
            self.const_to_value[self.value_to_const[value]] = value
        return self.value_to_const[value]

    def get_value(self, const):
        return self.const_to_value[const]

    def get_param(self):
        return EasyParameter(self)

    def __call__(self, *args):
        if len(args) == 0:
            return self.get_param()
        if len(args) == 1:
            return self.get_const(args[0])
        raise ValueError(args)


class Object(object):

    def __init__(self, name, type):
        """
        PDDL object abstract class.

        :param name: the string name of the object
        :param type: the :class:`.Type` of the object
        """
        self.name = name.lower()
        self.type = type
        self._hash = None

    def __eq__(self, other):
        return type(self) == type(other) and self.name == other.name and self.type == other.type

    def __ne__(self, other):
        return not(self == other)

    def __hash__(self):
        if self._hash is None:
            self._hash = hash((self.__class__, self.name, self.type))
        return self._hash

    def pddl(self):
        return self.name

    def typed_pddl(self):
        return TYPE_PAIR % (self.name, self.type)
    __repr__ = pddl


class Constant(Object):
    pass
    """
  PDDL constant which extends :class:`.Object`.
  """


class Parameter(Object):

    def __init__(self, name, type):
        """
        PDDL parameter which extends :class:`.Object`.

        :param name: the string name of the parameter
        :param type: the :class:`.Type` of the parameter
        """
        super(Parameter, self).__init__(PARAM_PREFIX + name, type)

    @property
    def original(self):
        return self.name[len(PARAM_PREFIX):]


class EasyParameter(Parameter):
    """
    Anonymous parameter which extends :class:`.Parameter` by automatically generating a unique name.

    .. code:: python

      CONF = EasyType()
      Q = EasyParameter(CONF)
    """
    _num = 0
    _template = 'x%s'

    def __init__(self, type):
        """
        :param type: the :class:`.Type` of the parameter
        """
        super(EasyParameter, self).__init__(
            self._template % EasyParameter._num, type)
        EasyParameter._num += 1


class NamedObject(Constant):

    def __init__(self, value):
        super(NamedObject, self).__init__(value.name, self.type)
        self.value = value


class StringObject(Constant):

    def __init__(self, value):
        super(StringObject, self).__init__('%s%s' %
                                           (self.prefix, value), self.type)
        self.value = value


class HashableObject(Constant):
    dictionary = {}
    value_repr = True

    def __init__(self, value):
        if self.prefix not in self.dictionary:
            self.dictionary[self.prefix] = {}
        if value not in self.dictionary[self.prefix]:
            self.dictionary[self.prefix][value] = '%s%s' % (
                self.prefix, len(self.dictionary[self.prefix]))
        super(HashableObject, self).__init__(
            self.dictionary[self.prefix][value], self.type)
        self.value = value

    def __repr__(self):
        if self.value_repr:
            repr_value = round(self.value, 3) if isinstance(
                self.value, float) else self.value
            return '%s(%s)' % (self.prefix, repr_value)
        return self.name
